//
// Created by yanhang on 5/13/17.
//

#include <math.h>
#include <random>

#include "plane_stereo.h"

#include "MRF2.2/mrf.h"
#include "MRF2.2/GCoptimization.h"

using namespace std;
using namespace cv;
using namespace Eigen;

namespace DPM {

std::vector<int> NonGroundIndex(const std::vector<Eigen::Vector3d> &vertices,
                                const double threshold) {
  Plane3D ground_plane(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1));
  std::vector<int> non_ground_index;
  for (int i = 0; i < vertices.size(); ++i) {
    if (ground_plane.getAbsoluteDistance(vertices[i]) > threshold) {
      non_ground_index.push_back(i);
    }
  }
  return non_ground_index;
}

void RemoveGround(std::vector<Eigen::Vector3d> &vertices,
                  const double threshold) {
  Plane3D ground_plane(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1));
  std::vector<Eigen::Vector3d> non_ground_vert;
  for (const auto &vert: vertices) {
    if (ground_plane.getAbsoluteDistance(vert) > threshold) {
      non_ground_vert.push_back(vert);
    }
  }
  if (non_ground_vert.size() < vertices.size() / 2) {
    printf("Ground removed. Number of non-ground vertices: %d/%d.\n",
           (int) non_ground_vert.size(),
           (int) vertices.size());
    vertices.swap(non_ground_vert);
  } else {
    printf("No ground found.\n");
  }
}

void GeneratePlanes(const std::vector<Eigen::Vector3d> &pc,
                    std::vector<Plane3D> &planes,
                    const int inner_iter,
                    const double threshold) {
  std::vector<Eigen::Vector3d> vertices = pc;

  int num_attemped = 0;
  constexpr int kMaxAttempt = 3;

  int iter = 0;
  while (vertices.size() > 3) {
    Plane3D cur_plane;
    std::vector<bool> is_inlier;
    bool is_fitted = plane_util::planeFromPointsRANSAC(vertices, cur_plane, is_inlier,
                                                       threshold, inner_iter);
    if (!is_fitted) {
      if (num_attemped == kMaxAttempt) {
        LOG(WARNING) << "Maximum attempts achieved, return";
        break;
      }
      LOG(WARNING) << "No plane fitted";
      ++num_attemped;
      continue;
    }
    num_attemped = 0;
    planes.push_back(cur_plane);
    std::vector<Eigen::Vector3d> new_vertices;
    for (auto i = 0; i < vertices.size(); ++i) {
      if (!is_inlier[i]) {
        new_vertices.push_back(vertices[i]);
      }
    }
    vertices.swap(new_vertices);
    printf("Iter %d, plane fitted: %d, number of vertices remained: %d\n", iter, static_cast<int>(planes.size()),
           static_cast<int>(vertices.size()));
    iter++;
  }
}

void SolvePlaneStereo(const std::vector<Eigen::Vector3d> &pt,
                      const std::vector<Plane3D> &planes,
                      std::vector<Eigen::Vector3d> &new_vertices,
                      std::vector<int> &plane_assignment,
                      const double lambda) {
  // construct problem
  const int nVariable = (int)pt.size();
  const int nLabel = (int) planes.size();

  // assign unary cost
  std::vector<double> unary(pt.size() * planes.size(), 0.0);

  // vid: vertex index; pid: plane index
  for (auto vid = 0; vid < pt.size(); ++vid) {
    for (auto pid = 0; pid < planes.size(); ++pid) {
      unary[planes.size() * vid + pid] = planes[pid].getAbsoluteVerticalDistance(pt[vid]);
      // unary[planes.size() * vid + pid] = planes[pid].getAbsoluteDistance(pt[vid]);
    }
  }

  // assign smoothness cost (Potts model)
  std::vector<double> pairwise(planes.size() * planes.size(), lambda);
  for (auto i = 0; i < planes.size(); ++i) {
    pairwise[i * nLabel + i] = 0;
  }

  DataCost *dataCost = new DataCost(unary.data());
  SmoothnessCost *smoothnessCost = new SmoothnessCost(pairwise.data());

  EnergyFunction *energy_function = new EnergyFunction(dataCost, smoothnessCost);
  std::shared_ptr<MRF> mrf(new Expansion(nVariable, nLabel, energy_function));
  // Set the neighboring system
  for (int i=0; i<pt.size(); ++i){
    for (int j=0; j<i; ++j){
      double dis_x_y = (pt[i].block<2, 1>(0, 0) - pt[j].block<2, 1>(0, 0)).norm();
      if (dis_x_y <= 1.1){
        mrf->setNeighbors(i, j, 1.0);
      }
    }
  }

  mrf->initialize();

  std::default_random_engine generator;
  std::uniform_int_distribution<int> distribution(0, nLabel - 1);
  for (auto i = 0; i < nVariable; ++i) {
    mrf->setLabel(i, distribution(generator));
  }

  double init_data_energy = mrf->dataEnergy();
  double init_smoothness_energy = mrf->smoothnessEnergy();

  constexpr int max_iter = 10;
  float time;
  printf("Solving Alpha-Expansion\n");
  mrf->optimize(max_iter, time);

  double final_data_energy = mrf->dataEnergy();
  double final_smoothness_energy = mrf->smoothnessEnergy();

  printf("Done. Time Usage: %.2fs. Init energy: %.3f (d:%.3f, s:%.3f), final energy: %.3f (d:%.3f, s:%.3f)\n",
         time, init_data_energy + init_smoothness_energy, init_data_energy, init_smoothness_energy,
         final_data_energy + final_smoothness_energy, final_data_energy, final_smoothness_energy);

  // store result
  new_vertices.resize(pt.size(), Eigen::Vector3d(0, 0, 0));
  plane_assignment.resize(pt.size(), 0);
  for (auto i = 0; i < nVariable; ++i) {
    const int pid = mrf->getLabel(i);
    plane_assignment[i] = pid;
    new_vertices[i] = planes[pid].projectFromWorldToPlaneVertical(pt[i]);
  }
}

}//namespace DPM