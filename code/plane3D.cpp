//
//  3Dplane.cpp
//  DynamicOptimize
//
//  Created by Yan Hang on 10/16/14.
//  Copyright (c) 2014 Washington Universtiy. All rights reserved.
//

#include "plane3D.h"
#include <time.h>
#include <random>

using namespace std;
using namespace Eigen;

namespace DPM {

double Plane3D::epsilon = 1e-4;

Plane3D::Plane3D(const Vector3d &p0, const Vector3d &p1, const Vector3d &p2) {
//  Matrix3d A;
//  Vector3d b(-1, -1, -1);
//  A.block<1, 3>(0, 0) = p0;
//  A.block<1, 3>(1, 0) = p1;
//  A.block<1, 3>(2, 0) = p2;
  CHECK (!plane_util::ThreePointsColinear(p0, p1, p2)) << "Colinear points"
                                                       << endl << "p0:" << p0
                                                       << endl << "p1:" << p1
                                                       << endl << "p2:" << p2;
  normal = (p1 - p0).cross(p2 - p0);
  // normal = A.inverse() * b;
  const double norm = normal.norm();
  if (norm <= epsilon) {
    printf("Bad condition: (%.5f,%.5f,%.5f),(%.5f,%.5f,%.5f),(%.5f,%.5f,%.5f)\n", p0[0], p0[1], p0[2],
           p1[0], p1[1], p1[2], p2[0], p2[1], p2[2]);
  }
  // CHECK_GT(norm, 0) << endl << A;
  normal = normal / norm;
  offset = -1 * normal.dot(p0);
  CHECK_LE(std::fabs(p1.dot(normal) + offset), epsilon);
  CHECK_LE(std::fabs(p2.dot(normal) + offset), epsilon);
  // offset = 1 / norm;
}

namespace plane_util {
void planeIntersection(const Plane3D &plane1,
                       const Plane3D &plane2,
                       Vector3d &normal,
                       Vector3d &pt) {
  Vector3d n1 = plane1.getNormal();
  Vector3d n2 = plane2.getNormal();
  normal = n1.cross(n2);
  CHECK_GT(normal.norm(), 0);

  normal.normalize();
  Matrix2d A;
  Vector2d p(-1 * plane1.getOffset(), -1 * plane2.getOffset());
  Vector2d xy;
  if (normal[2] != 0) {
    A << n1[0], n1[1], n2[0], n2[1];
    xy = A.inverse() * p;
    pt[0] = xy[0];
    pt[1] = xy[1];
    pt[2] = 0.0;
    return;
  } else if (normal[0] != 0) {
    A << n1[1], n1[2], n2[1], n2[2];
    xy = A.inverse() * p;
    pt[0] = 0.0;
    pt[1] = xy[0];
    pt[2] = xy[1];
    return;
  } else if (normal[1] != 0) {
    A << n1[0], n1[2], n2[0], n2[2];
    xy = A.inverse() * p;
    pt[0] = xy[0];
    pt[1] = 0.0;
    pt[2] = xy[1];
    return;
  }
}

bool planeFromPointsLeastSquare(const std::vector<Eigen::Vector3d>& pts, Plane3D &plane) {
  Eigen::VectorXd b(pts.size());
  b.setConstant(-1);
  Eigen::MatrixXd A(pts.size(), 3);
  for (int i=0; i<pts.size(); ++i){
    A.block<1, 3>(i, 0) = pts[i];
  }

  Eigen::Vector3d n = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);

  const double epsilon = std::numeric_limits<double>::epsilon();
  const double nn = n.norm();
  if (nn < epsilon) {
    return false;
  }
  n /= nn;
  plane.setNormal(n);
  plane.setOffset(1.0 / nn);
  return true;
}

bool planeFromPointsRANSAC(const std::vector<Eigen::Vector3d> &pts, Plane3D &plane,
                           std::vector<bool> &is_inlier,
                           const double dis_thres, const int max_iter) {
  size_t max_inlier = 0;
  const double epsilon = 1e-5;
  int N = (int) pts.size();
  CHECK_GE(N, 3);
  plane = Plane3D();

  std::default_random_engine generator;
  std::uniform_int_distribution<int> distribution(0, (int) pts.size() - 1);
  is_inlier.resize(pts.size(), false);
  for (int iter = 0; iter < max_iter; ++iter) {
    int id1 = distribution(generator);
    int id2 = distribution(generator);
    int id3 = distribution(generator);

    if (id1 == id2 || id1 == id3 || id2 == id3) {
      iter--;
      continue;
    }
    Eigen::Matrix3d A;
    A.block<1, 3>(0, 0) = pts[id1];
    A.block<1, 3>(1, 0) = pts[id2];
    A.block<1, 3>(2, 0) = pts[id3];
    if (plane_util::ThreePointsColinear(pts[id1], pts[id2], pts[id3])) {
      continue;
    }

    if (A.determinant() < std::numeric_limits<double>::epsilon()){
      continue;
    }

    Plane3D curplane(pts[id1], pts[id2], pts[id3]);
    vector<bool> cur_inlier(pts.size(), false);
    vector<Vector3d> inliers;
    inliers.reserve(pts.size());
    for (int i = 0; i < pts.size(); ++i) {
      double dis = curplane.getAbsoluteDistance(pts[i]);
      if (dis < dis_thres) {
        inliers.emplace_back(pts[i][0], pts[i][1], pts[i][2]);
        cur_inlier[i] = true;
      }
    }
    if (inliers.size() > max_inlier) {
      if (!planeFromPointsLeastSquare(inliers, plane)) {
        LOG(WARNING) << "Least square failed";
        continue;
      }
      max_inlier = inliers.size();
      is_inlier.swap(cur_inlier);
    }
  }
  return plane.getNormal().norm() >= epsilon;
}

}//namespace plane_util
}//namespace dynamic_stereo
