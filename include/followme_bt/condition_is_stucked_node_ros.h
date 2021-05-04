#ifndef CONDITIONISSTUCKEDNODEROS_H
#define CONDITIONISSTUCKEDNODEROS_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace follow_me_bt_condition_nodes{
class ConditionIsStuckedNode : public BT::ConditionNode
{
  public:
    
    ConditionIsStuckedNode(const std::string& name , const BT::NodeConfiguration & config):
    BT::ConditionNode(name, config)
    {
    expected_result_ = BT::NodeStatus::FAILURE;
    tick_count_ = 0;
    };

    void setExpectedResult(BT::NodeStatus res);

    // The method that is going to be executed by the thread
    virtual BT::NodeStatus tick() override;

    int tickCount() const
    {
        return tick_count_;
    };

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<int>("follow_status") };
    };


  private:
    BT::NodeStatus expected_result_;
    int tick_count_;
    int follow_status_;
};
}
#endif