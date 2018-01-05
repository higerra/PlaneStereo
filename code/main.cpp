#include "plane_stereo.h"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <glog/logging.h>
#include <gflags/gflags.h>

DEFINE_int32(RANSAC_max_iter, 1000, "The inner iterations of RANSAC");
DEFINE_double(RANSAC_threshold, 0.3, "The inlier threshold of RANSAC");
DEFINE_bool(has_ground, false, "If set to true, a ground plane will be computed by 3 corner points and removed");

using namespace std;

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
  std::vector<Eigen::Vector3d> vertices;
  std::vector<int> non_ground_index;
  if (FLAGS_has_ground){
    non_ground_index = DPM::NonGroundIndex(mesh.vertices, FLAGS_RANSAC_threshold);
    for (int idx: non_ground_index) {
      vertices.push_back(mesh.vertices[idx]);
    }
  } else {
    vertices = mesh.vertices;
    for (int i=0; i<vertices.size(); ++i){
      non_ground_index.push_back(i);
    }
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

  DPM::Mesh result_mesh = mesh;
  for (int i=0; i<non_ground_index.size(); ++i){
    result_mesh.vertices[non_ground_index[i]] = new_vertices[i];
  }

  result_mesh.Write(argv[2]);
  return 0;
}
