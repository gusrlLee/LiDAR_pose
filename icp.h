#include "Eigen/Eigen"
#include <vector>
#include "config.h"

#ifndef ICP_H
#define ICP_H

typedef struct{
    Eigen::Matrix3d trans;
    std::vector<float> distances;
    int iter;
}  ICP_OUT;

typedef struct{
    std::vector<float> distances;
    std::vector<int> indices;
} NEIGHBOR;

Eigen::Matrix3d best_fit_transform(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B);

ICP_OUT icp(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, int max_iterations=20, int tolerance = 0.001);

// throughout method
NEIGHBOR nearest_neighbot(const Eigen::MatrixXd &src, const Eigen::MatrixXd &dst);
float dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb);

#endif
