#ifndef POSEOPTIMIZER_H
#define POSEOPTIMIZER_H

#include "walking_controller/a1_adapter.h"
#include "walking_controller/qp_array.h"
#include "walking_controller/QuadProg++.h"

#include "util_transformations.h"

#include "kindr/Core"
#include "grid_map_core/Polygon.hpp"

#include <unordered_map>
#include <map>
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "Eigen/Core"
#include "Eigen/SVD"

typedef QuadrupedDescription::LimbList LimbList;
const std::vector<LimbList> limbEnumCounterClockWiseOrder = { LimbList::LF_LEG,
                                                              LimbList::LH_LEG,
                                                              LimbList::RH_LEG,
                                                              LimbList::RF_LEG };

struct EnumClassHash
{
    template<typename T>
    std::size_t operator()(T t) const
    {
        return static_cast<std::size_t>(t);
    }
};

struct CompareByCounterClockwiseOrder
{
  bool operator()(const LimbList& a, const LimbList& b) const
  {
    const size_t aIndex = findIndex(a);
    const size_t bIndex = findIndex(b);
    return aIndex < bIndex;
  }

  size_t findIndex(const LimbList& limb) const
  {
    auto it = std::find(limbEnumCounterClockWiseOrder.begin(), limbEnumCounterClockWiseOrder.end(), limb);
    if (it == limbEnumCounterClockWiseOrder.end()) {
      throw std::runtime_error("Could not find index for limb!");
    }
    return std::distance(limbEnumCounterClockWiseOrder.begin(), it);
  }
};

class PoseOptimizer
{
    public:
        PoseOptimizer();
        Eigen::VectorXd optimize(std::array<Eigen::Matrix4f, 4> foot_positions, bool foot_contacts[4], float roll, float pitch, float yaw);
        void minimize(Eigen::VectorXd& params);
    private:
        void setGlobalHessian(Eigen::MatrixXd& hessian);
        void setLinearTerm(Eigen::VectorXd& jacobian);
        void setGlobalInequalityConstraintJacobian(Eigen::MatrixXd& A);
        void setInequalityConstraintMaxValues(Eigen::VectorXd& b);
        void setGlobalEqualityConstraintJacobian(Eigen::MatrixXd& Aeq);
        void setEqualityConstraintMaxValues(Eigen::VectorXd& beq);

        // typedef std::unordered_map<LimbEnum, Position, EnumClassHash> Stance;
        quadprogpp::Matrix<double> G_;
        quadprogpp::Vector<double> g0_;
        quadprogpp::Matrix<double> CE_;
        quadprogpp::Vector<double> ce0_;
        quadprogpp::Matrix<double> CI_;
        quadprogpp::Vector<double> ci0_;
};

#endif