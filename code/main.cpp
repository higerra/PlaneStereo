#include "plane_stereo.h"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <glog/logging.h>
#include <gflags/gflags.h>

DEFINE_int32(RANSAC_max_iter, 1000, "The inner iterations of RANSAC");
DEFINE_double(RANSAC_threshold, 0.5, "The inlier threshold of RANSAC");

using namespace std;

int main(int argc, char **argv) {
  if (argc < 3) {
    cerr << "Usage ./PlaneStereo <path-to-file> <output-path>" << endl;
    return 1;
  }
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;

  // Test with distance
  DPM::Plane3D test_plane(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 1));
  DPM::Plane3D test_plane2(Eigen::Vector3d(-1, -1, 1), Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(1, -1, 1));

//  printf("From point/normal: (%f, %f, %f), %f\n", test_plane.getNormal()[0], test_plane.getNormal()[1],
//         test_plane.getNormal()[2], test_plane.getOffset());
//  printf("From 3 points: (%f, %f, %f), %f\n", test_plane2.getNormal()[0], test_plane2.getNormal()[1],
//         test_plane2.getNormal()[2], test_plane2.getOffset());
//  Eigen::Vector3d test_pt(1, 1, 10);
//  printf("Vertical distance: %f\n", test_plane2.getSignedVerticalDistance(test_pt));
//  printf("Euclidean distance: %f\n", test_plane2.getSignedDistance(test_pt));
//
//  Eigen::Vector3d pt_on_plane = test_plane.projectFromWorldToPlaneVertical(test_pt);
//  Eigen::Vector3d pt_on_plane2 = test_plane.projectFromWorldToPlane(test_pt);
//  cout << "Projected from vertical distance: " << pt_on_plane.transpose() << endl;
//  cout << "Projected from distance: " << pt_on_plane2.transpose() << endl;
//  exit(-1);

  DPM::Mesh mesh;
  CHECK(mesh.Read(std::string(argv[1]))) << "Can not read mesh: " << argv[1];
  std::vector<Eigen::Vector3d> vertices = mesh.vertices;

  printf("Total number of vertices: %d\n", (int) vertices.size());

  // std::vector<int> non_ground_index = DPM::NonGroundIndex(vertices, threshold);
  std::vector<int> non_ground_index;
  for (int i=0; i<vertices.size(); ++i){
    non_ground_index.push_back(i);
  }
  printf("Number of vertices: %d\n", (int) vertices.size());
  // Generate plane proposals by RANSAC
  std::vector<DPM::Plane3D> planes;
  printf("Fitting plane...\n");
  DPM::GeneratePlanes(vertices, planes, FLAGS_RANSAC_max_iter, FLAGS_RANSAC_threshold);
  printf("%d planes fitted\n", (int) planes.size());

  std::vector<Eigen::Vector3d> new_vertices;
  std::vector<int> plane_assignment;
  printf("Solving...\n");
  DPM::SolvePlaneStereo(vertices, planes, new_vertices, plane_assignment, 5.0);

  DPM::Mesh result_mesh;
  for (int vid: non_ground_index) {
    result_mesh.vertices.push_back(new_vertices[vid]);
  }
  result_mesh.faces = mesh.faces;
  result_mesh.Write(argv[2]);

  return 0;
}
