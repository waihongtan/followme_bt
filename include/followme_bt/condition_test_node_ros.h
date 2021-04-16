#ifndef CONDITIONTESTROS_H
#define CONDITIONTESTROS_H

#include "behaviortree_cpp_v3/condition_node.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace BT
{
class ConditionIsTrackedNode : public ConditionNode
{
  public:
    
    ConditionIsTrackedNode(const std::string& name , const NodeConfiguration & config,ros::NodeHandle* nh);

    void setExpectedResult(NodeStatus res);

    // The method that is going to be executed by the thread
    virtual BT::NodeStatus tick() override;

    int tickCount() const
    {
        return tick_count_;
    };

    static PortsList providedPorts()
    {
        return { BT::OutputPort<geometry_msgs::PoseStamped>("output_goal") };
    };


  private:
    NodeStatus expected_result_;
    int tick_count_;
    ros::NodeHandle nh_ptr_;
};
}

#endif
