#ifndef CONDITION_ISOSCILLATING_ROS_H
#define CONDITION_ISOSCILLATING_ROS_H


#include "behaviortree_cpp_v3/condition_node.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace BT
{
class ConditionIsOscillatingNode : public ConditionNode
{
  public:
    
    ConditionIsOscillatingNode(const std::string& name , const NodeConfiguration & config,ros::NodeHandle* nh);

    void setExpectedResult(NodeStatus res);

    // The method that is going to be executed by the thread
    virtual BT::NodeStatus tick() override;

    int tickCount() const
    {
        return tick_count_;
    };

    static PortsList providedPorts()
    {
        return { BT::InputPort<int>("follow_status") };
    };


  private:
    NodeStatus expected_result_;
    int tick_count_;
    ros::NodeHandle nh_ptr_;
    int follow_status_ = 0;
};
}

#endif
