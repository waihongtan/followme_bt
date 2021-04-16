#ifndef ACTIONMOVEBACKWARDS_H
#define ACTIONMOVEBACKWARDS_H

#include "behaviortree_cpp_v3/action_node.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

namespace BT
{
class AsyncActionMoveBackwards : public AsyncActionNode
{
  public:
    AsyncActionMoveBackwards(const std::string& name , const NodeConfiguration & config, ros::NodeHandle* nh, BT::Duration deadline_ms = std::chrono::milliseconds(100));

    virtual ~AsyncActionMoveBackwards()
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

    // void  callback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg);
    void resetParameters();
    static PortsList providedPorts()
    {
        return { 
                 BT::InputPort<int>("global_costmap_gradient"),
                 BT::InputPort<int>("local_costmap_gradient") };
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
    ros::Subscriber goal_sub_;
    dynamic_reconfigure::ReconfigureRequest srv_req_;
    dynamic_reconfigure::ReconfigureResponse srv_resp_;
    dynamic_reconfigure::DoubleParameter param_double_;
    dynamic_reconfigure::IntParameter param_;
    // dynamic_reconfigure::IntParameter * param_ptr_;
    dynamic_reconfigure::Config conf_;
    int initial_global_gradient_;
    int initial_local_gradient_;
    double initial_max_speed_;
    geometry_msgs::PoseStamped goal_;
    std_msgs::Int32 status_;
};
}

#endif
