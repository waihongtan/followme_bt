#ifndef ACTIONTESTROS_H
#define ACTIONTESTROS_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>

namespace BT
{
class AsyncActionPlanNormally : public AsyncActionNode
{
  public:
    AsyncActionPlanNormally(const std::string& name , const NodeConfiguration & config, ros::NodeHandle* nh, BT::Duration deadline_ms = std::chrono::milliseconds(100));

    virtual ~AsyncActionPlanNormally()
    {
        halt();
    }

    // The method that is going to be executed by the thread
    BT::NodeStatus tick() override;

    void setTime(BT::Duration time);

    // The method used to interrupt the execution of the node
    virtual void halt() override;

    void setExpectedResult(NodeStatus res);

    int tickCount() const {
        return tick_count_;
    }

    int successCount() const {
        return success_count_;
    }

    int failureCount() const {
        return failure_count_;
    }

    void resetCounters() {
        success_count_ = 0;
        failure_count_ = 0;
        tick_count_ = 0;
    }

    static PortsList providedPorts()
    {
        return { BT::InputPort<geometry_msgs::PoseStamped>("input_goal"),
                 BT::OutputPort<int>("follow_status") };
    }

  private:
    // using atomic because these variables might be accessed from different threads
    BT::Duration time_;
    std::atomic<NodeStatus> expected_result_;
    std::atomic<int> tick_count_;
    int success_count_;
    int failure_count_;
    ros::NodeHandle nh_ptr_;
    ros::Publisher goal_pub_;

};
}

#endif
