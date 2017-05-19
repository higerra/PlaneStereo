//
// Created by yanhang on 5/13/17.
//

#include <math.h>

#include "plane_stereo.h"

#include "MRF2.2/mrf.h"
#include "MRF2.2/GCoptimization.h"

using namespace std;
using namespace cv;
using namespace Eigen;

namespace DPM{

    void RemoveGround(std::vector<Eigen::Vector3d>& vertices,
                      const double threshold){
        Plane3D ground_plane(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1));
        std::vector<Eigen::Vector3d> non_ground_vert;
        for(const auto& vert: vertices){
            if(ground_plane.getDistance(vert) > threshold){
                non_ground_vert.push_back(vert);
            }
        }
        if(non_ground_vert.size() < vertices.size() / 2){
            printf("Ground removed. Number of non-ground vertices: %d/%d.\n", (int)non_ground_vert.size(), (int)vertices.size());
            vertices.swap(non_ground_vert);
        }else{
            printf("No ground found.\n");
        }
    }

    void GeneratePlanes(const std::vector<Eigen::Vector3d>& pc,
                        std::vector<Plane3D>& planes,
                        cv::Mat& color_map,
                        const int inner_iter,
                        const double threshold){
        std::vector<Eigen::Vector3d> vertices = pc;
        int W = (int)sqrt((double) pc.size());

        int num_attemped = 0;
        constexpr int kMaxAttempt = 3;

        std::vector<cv::Vec3b> colors{cv::Vec3b(255, 0, 0), cv::Vec3b(0, 255, 0), cv::Vec3b(0, 0, 255),
                                      cv::Vec3b(128, 255, 0), cv::Vec3b(128, 0, 255), cv::Vec3b(0, 128, 255),
                                      cv::Vec3b(255, 128, 0), cv::Vec3b(255, 0, 128), cv::Vec3b(0, 255, 128)};

        color_map = cv::Mat(W, W, CV_8UC3, cv::Scalar(255, 255, 255));

        RemoveGround(vertices, threshold);
        int iter = 0;
        while(vertices.size() > 3){
            Plane3D cur_plane;
            std::vector<bool> is_inlier;
            bool is_fitted = plane_util::planeFromPointsRANSAC(vertices, cur_plane, is_inlier,
                                                               threshold, inner_iter);
            if(!is_fitted){
                if(num_attemped == kMaxAttempt){
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
            for(auto i=0; i<vertices.size(); ++i){
                if(is_inlier[i]){
                    color_map.at<cv::Vec3b>((int)vertices[i][1], (int)vertices[i][0]) = colors[(int)planes.size() % (int)colors.size()];
                }else{
                    new_vertices.push_back(vertices[i]);
                }
            }
            vertices.swap(new_vertices);

            printf("Iter %d, plane fitted: %d, number of vertices remained: %d\n", iter, (int)planes.size(), (int)vertices.size());
            iter++;
        }
    }

    void SolvePlaneStereo(const std::vector<Eigen::Vector3d>& pt,
                          const std::vector<Plane3D>& planes,
                          std::vector<Eigen::Vector3d>& new_vertices,
                          std::vector<int>& plane_assignment,
                          const double lambda){
        // construct problem
        const int W = (int)sqrt((double)pt.size());
        const int nLabel = (int)planes.size();

        // assign unary cost
        std::vector<double> unary(pt.size() * planes.size(), 0.0);

        // vid: vertex index; pid: plane index
        for(auto vid=0; vid<pt.size(); ++vid){
            for(auto pid=0; pid<planes.size(); ++pid){
                unary[planes.size() * vid + pid] = planes[pid].getDistance(pt[vid]);
            }
        }

        // assign smoothness cost (Potts model)
        std::vector<double> pairwise(planes.size() * planes.size(), lambda);
        for(auto i=0; i<planes.size(); ++i){
            pairwise[i * nLabel + i] = 0;
        }

        DataCost *dataCost = new DataCost(unary.data());
        SmoothnessCost *smoothnessCost = new SmoothnessCost(pairwise.data());

        EnergyFunction *energy_function = new EnergyFunction(dataCost, smoothnessCost);
        std::shared_ptr<MRF> mrf(new Expansion(W, W, nLabel, energy_function));

        mrf->initialize();

        std::default_random_engine generator;
        std::uniform_int_distribution<int> distribution(0, nLabel - 1);
        for(auto i=0; i < W * W; ++i){
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
               time, init_data_energy+init_smoothness_energy, init_data_energy, init_smoothness_energy,
               final_data_energy+final_smoothness_energy, final_data_energy, final_smoothness_energy);

        // store result
        new_vertices.resize(pt.size(), Eigen::Vector3d(0, 0, 0));
        plane_assignment.resize(pt.size(), 0);
        for(auto i=0; i<W*W; ++i){
            const int pid = mrf->getLabel(i);
            plane_assignment[i] = pid;
            new_vertices[i] = planes[pid].projectFromeWorldtoPlane(pt[i]);
        }
    }

}//namespace DPM