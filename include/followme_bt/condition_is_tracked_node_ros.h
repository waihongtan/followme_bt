#ifndef CONDITIONISTRACKEDNODEROS_H
#define CONDITIONISTRACKEDNODEROS_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace follow_me_bt_condition_nodes{
class ConditionIsTrackedNode : public BT::ConditionNode
{
  public:
    
    ConditionIsTrackedNode(const std::string& name , const BT::NodeConfiguration & config, const std::shared_ptr<ros::NodeHandle>& nh):
    BT::ConditionNode(name, config),
    nh_(nh) {
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
        return { BT::OutputPort<geometry_msgs::PoseStamped>("output_goal") };
    };


  private:
    BT::NodeStatus expected_result_;
    int tick_count_;
    std::shared_ptr<ros::NodeHandle> nh_;
};
}
#endif
