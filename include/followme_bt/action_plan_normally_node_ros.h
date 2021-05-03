#ifndef ACTIONPLANNORMALLYNODEROS_H
#define ACTIONPLANNORMALLYNODEROS_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>

namespace follow_me_bt_action_nodes{

class SyncActionPlanNormally : public BT::SyncActionNode
{
  public:
    SyncActionPlanNormally(const std::string& name , const BT::NodeConfiguration & config, const std::shared_ptr<ros::NodeHandle>& nh):
    BT::SyncActionNode(name, config), 
    nh_(nh)
    {
        expected_result_ = BT::NodeStatus::FAILURE;
        tick_count_ = 0;
        goal_pub_ = nh_->advertise<geometry_msgs::PoseStamped>("/global_goal_bt",1);
        goal_sub_ = nh_->subscribe("/global_goal", 1, &SyncActionPlanNormally::goalCallBack,this);
        status_sub_ = nh_->subscribe("/follow_status", 1, &SyncActionPlanNormally::statusCallback,this);
    };

    // The method that is going to be executed by the thread
    BT::NodeStatus tick() override;

    void goalCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
      goal_ = *msg;
      last_received_goal_ = ros::Time::now().toSec();
    };

    void statusCallback(const std_msgs::Int32::ConstPtr& msg)
    {
      status_ = *msg;
      last_received_status_ = ros::Time::now().toSec();
    };

    void setExpectedResult(BT::NodeStatus res)
    {

    expected_result_ = res;
    
    };    

    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<int>("follow_status") };
    }

  private:
    // using atomic because these variables might be accessed from different threads
    std::atomic<BT::NodeStatus> expected_result_;
    std::atomic<int> tick_count_;
    std::shared_ptr<ros::NodeHandle> nh_;   
    ros::Publisher goal_pub_;
    ros::Subscriber goal_sub_ ; 
    ros::Subscriber status_sub_ ; 
    geometry_msgs::PoseStamped goal_;
    std_msgs::Int32 status_;
    double last_received_goal_ = 0;
    double last_received_status_ = 0;


};
};

#endif
