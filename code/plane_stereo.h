//
// Created by yanhang on 5/13/17.
//

#ifndef PLANESTEREO_PLANE_STEREO_H
#define PLANESTEREO_PLANE_STEREO_H

#include <iostream>
#include <string>
#include <memory>

#include <opencv2/opencv.hpp>
#include <glog/logging.h>
#include "mesh.h"
#include "plane3D.h"

namespace DPM {

std::vector<int> NonGroundIndex(const std::vector<Eigen::Vector3d> &vertices,
                                const double threshold);

void RemoveGround(std::vector<Eigen::Vector3d> &vertices,
                  const double threshold);

void GeneratePlanes(const std::vector<Eigen::Vector3d> &pc,
                    std::vector<Plane3D> &planes,
                    const int inner_iter = 1000,
                    const double threshold = 1);

void SolvePlaneStereo(const std::vector<Eigen::Vector3d> &pt,
                      const std::vector<Plane3D> &planes,
                      std::vector<Eigen::Vector3d> &new_vertices,
                      std::vector<int> &plane_assignment,
                      const double lambda = 1.0);
} //namespace DPM


#endif //PLANESTEREO_PLANE_STEREO_H
