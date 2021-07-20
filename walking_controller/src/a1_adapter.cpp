#include <walking_controller/a1_adapter.h>

A1Adapter::A1Adapter():
    // AdapterBase(),
    worldFrameId_("odom"),  // const std::string worldFrameId_;
    baseFrameId_("base")    // const std::string baseFrameId_;
 {
    // state_.reset(new State()); // std::unique_ptr<State> state_;
    // state_->initialize(getLimbs(), getBranches());

    // TODO: Define LimbEnum, BranchEnum
    limbs_.push_back(QuadrupedDescription::LimbList::LF_LEG);
    limbs_.push_back(QuadrupedDescription::LimbList::RF_LEG);
    limbs_.push_back(QuadrupedDescription::LimbList::LH_LEG);
    limbs_.push_back(QuadrupedDescription::LimbList::RH_LEG);

    branches_.push_back(QuadrupedDescription::BranchList::BASE);
    branches_.push_back(QuadrupedDescription::BranchList::LF_LEG);
    branches_.push_back(QuadrupedDescription::BranchList::RF_LEG);
    branches_.push_back(QuadrupedDescription::BranchList::LH_LEG);
    branches_.push_back(QuadrupedDescription::BranchList::RH_LEG);
}

const std::vector<QuadrupedDescription::LimbList>& A1Adapter::getLimbs() const {
    return limbs_;
}

const std::vector<QuadrupedDescription::BranchList>& A1Adapter::getBranches() const {
    return branches_;
}
