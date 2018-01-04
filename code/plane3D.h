#ifndef PLANE_3D_H
#define PLANE_3D_H

#include <iostream>
#include <Eigen/Eigen>
#include <glog/logging.h>

namespace DPM {
class Plane3D {

 public:
  Plane3D() : normal(0, 0, 0) {}

  Plane3D(const Eigen::Vector3d &, const Eigen::Vector3d &, const Eigen::Vector3d &);

  Plane3D(const Eigen::Vector3d &pt_, const Eigen::Vector3d &normal_) : normal(normal_) {
    CHECK_LT(normal_.norm() - 1.0, epsilon);
    offset = -1 * pt_.dot(normal);
  }

  inline void normalize() {
    double norm = normal.norm();
    CHECK_GT(norm, 0);
    normal = normal / norm;
    offset = offset / norm;
  }

  //get and set
  inline const Eigen::Vector3d &getNormal() const { return normal; }

  inline double getOffset() const { return offset; }

  void setNormal(const Eigen::Vector3d &n) { normal = n; }

  void setOffset(double o) { offset = o; }

  double getSignedDistance(const Eigen::Vector3d &pt) const{
    double nn = getNormal().norm();
    CHECK_GE(nn, epsilon);
    return (pt.dot(normal) + offset) / nn;
  }

  double getAbsoluteDistance(const Eigen::Vector3d& pt) const{
    return std::fabs(getSignedDistance(pt));
  }

  double getSignedVerticalDistance(const Eigen::Vector3d& pt) const{
    CHECK_NE(normal[2], 0) << "The plane is vertical.";
    return pt[2] + (pt[0] * normal[0] + pt[1] * normal[1] + offset) / normal[2];
  }

  double getAbsoluteVerticalDistance(const Eigen::Vector3d& pt) const{
    return std::fabs(getSignedVerticalDistance(pt));
  }

  Eigen::Vector3d projectFromWorldToPlane(const Eigen::Vector3d& world_pt) const{
    return world_pt - normal * getSignedDistance(world_pt);
  }

  Eigen::Vector3d projectFromWorldToPlaneVertical(const Eigen::Vector3d& world_pt) const{
    return world_pt - Eigen::Vector3d(0, 0, getSignedVerticalDistance(world_pt));
  }

 private:
  Eigen::Vector3d normal;
  double offset;
  static double epsilon;
};

namespace plane_util {
void planeIntersection(const Plane3D &plane1,
                       const Plane3D &plane2,
                       Eigen::Vector3d &normal,
                       Eigen::Vector3d &pt);
bool PlaneFromPointsLeastSquare(const std::vector<Eigen::Vector3d> &pts, Plane3D &plane);
bool planeFromPointsRANSAC(const std::vector<Eigen::Vector3d> &pts, Plane3D &plane,
                           std::vector<bool> &is_inlier,
                           const double dis_thres, const int max_iter = 500, bool verbose = false);

inline bool ThreePointsColinear(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3){
  return ((p2 - p1).cross(p1 - p3)).norm() < std::numeric_limits<double>::epsilon();
}

}

}
#endif