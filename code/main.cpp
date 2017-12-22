#include "plane_stereo.h"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

using namespace std;

std::vector<cv::Vec3b> colors{cv::Vec3b(255, 0, 0), cv::Vec3b(0, 255, 0), cv::Vec3b(0, 0, 255),
                              cv::Vec3b(128, 255, 0), cv::Vec3b(128, 0, 255), cv::Vec3b(0, 128, 255),
                              cv::Vec3b(255, 128, 0), cv::Vec3b(255, 0, 128), cv::Vec3b(0, 255, 128)};

int main(int argc, char **argv) {
  if (argc < 3) {
    cerr << "Usage ./PlaneStereo <path-to-file> <output-path>" << endl;
    return 1;
  }
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;

  DPM::Mesh mesh;
  CHECK(mesh.Read(std::string(argv[1]))) << "Can not read mesh: " << argv[1];
  std::vector<Eigen::Vector3d> vertices = mesh.vertices;

  // Read mesh with OpenMesh
//  OpenMesh::TriMesh_ArrayKernelT<> mesh;
//  CHECK(OpenMesh::IO::read_mesh(mesh, argv[1])) << "Can not read mesh: " << argv[1];
//  std::vector<Eigen::Vector3d> vertices;
//  for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it){
//    auto pt = mesh.point(v_it);
//    vertices.emplace_back(pt[0], pt[1], pt[2]);
//  }

  printf("Total number of vertices: %d\n", (int) vertices.size());

  const int W = (int) sqrt((double) vertices.size());

  constexpr int max_iter = 100;
  constexpr double threshold = 0.5;

  // std::vector<int> non_ground_index = DPM::NonGroundIndex(vertices, threshold);
  std::vector<int> non_ground_index;
  for (int i=0; i<vertices.size(); ++i){
    non_ground_index.push_back(i);
  }
  printf("Number of vertices: %d\n", (int) vertices.size());
  // Generate plane proposals by RANSAC
  std::vector<DPM::Plane3D> planes;
  cv::Mat plane_map;
  printf("Fitting plane...\n");
  DPM::GeneratePlanes(vertices, planes, plane_map, max_iter, threshold);
  printf("%d planes fitted\n", (int) planes.size());

//    cv::resize(plane_map, plane_map, cv::Size(500, 500), 0, 0, cv::INTER_NEAREST);
//    cv::imshow("plane_map", plane_map);
//    cv::waitKey(0);

  std::vector<Eigen::Vector3d> new_vertices;
  std::vector<int> plane_assignment;
  printf("Solving...\n");
  DPM::SolvePlaneStereo(vertices, planes, new_vertices, plane_assignment, 5.0);

//    cv::Mat new_plane_map(W, W, CV_8UC3, cv::Scalar(255, 255, 255));
//    for(auto i=0; i<W*W; ++i){
//        new_plane_map.at<cv::Vec3b>(i/W, i%W) = colors[plane_assignment[i] % (int)colors.size()];
//    }
//    cv::resize(new_plane_map, new_plane_map, cv::Size(500, 500), 0, 0, cv::INTER_NEAREST);
//    cv::imshow("Plane assignment", new_plane_map);
//    cv::waitKey(0);

//  DPM::Mesh result_mesh;
//  for (int vid: non_ground_index) {
//    result_mesh.vertices.push_back(new_vertices[vid]);
//  }
//  result_mesh.Write(argv[2]);

  return 0;
}
