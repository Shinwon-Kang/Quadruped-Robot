#ifndef A1ADAPTER_H
#define A1ADAPTER_H

#include "vector"
#include "string"
#include <memory>

#include "kindr/Core"

class QuadrupedDescription
{
    private:
        // todo.

    public:
        enum class LimbList {
            LF_LEG,
            RF_LEG,
            RH_LEG,
            LH_LEG,
        };

        enum class BranchList {
            BASE,
            LF_LEG,
            RF_LEG,
            LH_LEG,
            RH_LEG,
        };
};

class A1Adapter
{
    public:
        A1Adapter();
        const std::vector<QuadrupedDescription::LimbList>& getLimbs() const;
        const std::vector<QuadrupedDescription::BranchList>& getBranches() const;

        kindr::Position3D getPositionbaseToHipInBaseFrame(const QuadrupedDescription::LimbList& limb) const {
            switch (limb) {
                case QuadrupedDescription::LimbList::LF_LEG:
                    return kindr::Position3D(0.1805, 0.047, 0.0);
                case QuadrupedDescription::LimbList::RF_LEG:
                    return kindr::Position3D(0.1805, -0.047, 0.0);
                case QuadrupedDescription::LimbList::LH_LEG:
                    return kindr::Position3D(-0.1805, 0.047, 0.0);
                case QuadrupedDescription::LimbList::RH_LEG:
                    return kindr::Position3D(-0.1805, -0.047, 0.0);
                // default:
                    // ERROR
            }
        }

    private:
        // std::unique_ptr<State> state_;

        const std::string worldFrameId_;
        const std::string baseFrameId_;

        std::vector<QuadrupedDescription::LimbList> limbs_;
        std::vector<QuadrupedDescription::BranchList> branches_;
};

#endif