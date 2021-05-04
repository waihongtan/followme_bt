#include "followme_bt/condition_is_oscillating_node_ros.h"

namespace follow_me_bt_condition_nodes{
BT::NodeStatus ConditionIsOscillatingNode::tick()
{
    this->setExpectedResult(BT::NodeStatus::FAILURE);
    getInput<int>("follow_status",follow_status_);
    if (follow_status_ == 2) {
        ConditionIsOscillatingNode::setExpectedResult(BT::NodeStatus::SUCCESS);
    }
return expected_result_;
};

void ConditionIsOscillatingNode::setExpectedResult(BT::NodeStatus res)
{
    expected_result_ = res;
};
};
