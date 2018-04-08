#ifndef KF_STATE_H
#define KF_STATE_H

#include <Eigen/Dense>

namespace kf
{
template <int stateCount>
struct State
{
    using ValueVector = Eigen::Matrix<double, stateCount, 1>;
    using CovMat = Eigen::Matrix<double, stateCount, stateCount>;
    ValueVector State;
    CovMat Covariance;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}

#endif
