#ifndef PLANNORMALLY_BT_NODES_H
#define PLANNORMALLY_BT_NODES_H
#include <ros/ros.h>
#include "behaviortree_cpp_v3/bt_factory.h"
#include <geometry_msgs/PoseStamped.h>
using namespace BT;

template <> inline

class PlanNormallyAction: public CoroActionNode
{
  public:
    PlanNormallyAction(const std::string& name):
        CoroActionNode(name, {})
    {}

    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<geometry_msgs::PoseStamped>("goal") };
    }

  private:
    NodeStatus tick() override;
    virtual void halt() override;
    void cleanup(bool halted);.
    geometry_msgs::PoseStamped goal_;
    std_msgs::Int32ConstPtr msg_;
    ros::Publisher goal_pub_;


}
