#ifndef KF_STATE_H
#define KF_STATE_H

namespace kf
{
template <int stateCount>
struct State
{
    using ValueVector = Eigen::Matrix<double, stateCount, 1>;
    using CovMat = Eigen::Matrix<double, stateCount, stateCount>;
    ValueVector State;
    CovMat Covariance;
};
}

#endif
