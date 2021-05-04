#include "followme_bt/condition_is_stucked_node_ros.h"

namespace follow_me_bt_condition_nodes{
BT::NodeStatus ConditionIsStuckedNode::tick()
{
    this->setExpectedResult(BT::NodeStatus::FAILURE);
    getInput<int>("follow_status",follow_status_);
    if (follow_status_ == 0) {
        ConditionIsStuckedNode::setExpectedResult(BT::NodeStatus::SUCCESS);
    }
return expected_result_;
};

void ConditionIsStuckedNode::setExpectedResult(BT::NodeStatus res)
{
    expected_result_ = res;
};
};
