#ifndef ACTIONDEFLATEANDPLANBACKWARDSNODEROS_H
#define ACTIONDEFLATEANDPLANBACKWARDSNODEROS_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>


namespace follow_me_bt_action_nodes{

class SyncActionDeflateAndPlanBackwards : public BT::SyncActionNode
{
  public:
    SyncActionDeflateAndPlanBackwards(const std::string& name , const BT::NodeConfiguration & config, const std::shared_ptr<ros::NodeHandle>& nh):
    BT::SyncActionNode(name, config), 
    nh_(nh)
    {
        expected_result_ = BT::NodeStatus::FAILURE;
        tick_count_ = 0;
        goal_pub_ = nh_->advertise<geometry_msgs::PoseStamped>("/global_goal_bt",1);
        status_sub_ = nh_->subscribe("/follow_status", 1, &SyncActionDeflateAndPlanBackwards::statusCallback,this);
        goal_.header.frame_id = "base_link";
        goal_.pose.position.x = -1.5;
        goal_.pose.position.y = 0.0;
    };

    // The method that is going to be executed by the thread
    BT::NodeStatus tick() override;

    void statusCallback(const std_msgs::Int32::ConstPtr& msg)
    {
      status_ = *msg;
      last_received_status_ = ros::Time::now().toSec();
    };

    void getInitialParams()
    {
        if (ros::param::get("/follow_me/max_speed", initial_max_speed_))
        {
            ROS_INFO("Got max speed param: %f", initial_max_speed_);
        }

        else 
        {
            ROS_ERROR("Failed to get param '/follow_me/max_speed'");
        };

        if (ros::param::get("/follow_me/costmap_gradient", initial_local_costmap_gradient_))
        {
            ROS_INFO("Got costmap param: %d", initial_local_costmap_gradient_);
        }

        else 
        {
            ROS_ERROR("Failed to get param '/follow_me/costmap_gradient'");
        };

        if (ros::param::get("/A_star_local_planner/costmap_gradient", initial_global_costmap_gradient_))
        {
            ROS_INFO("Got costmap param: %d", initial_global_costmap_gradient_);
        }

        else 
        {
            ROS_ERROR("Failed to get param '/A_star_local_planner/costmap_gradient'");
        };
       
    };

    void setBackwardsParams()
    {
        dynamic_reconfigure::DoubleParameter param_double;
        dynamic_reconfigure::IntParameter param_int;
        dynamic_reconfigure::Config conf;
        param_int.name = "costmap_gradient";
        param_int.value = 0;  
        conf.ints.push_back(param_int);
        srv_req_.config = conf;
        ros::service::call("A_star_local_planner/set_parameters",srv_req_,srv_resp_);
        ROS_INFO("A* Costmap deflated");

        param_double.name = "max_speed";
        param_double.value = 0.0;
        conf.doubles.push_back(param_double);
        param_double.name = "min_speed";
        param_double.value = -0.5;  
        conf.doubles.push_back(param_double);
        srv_req_.config = conf;
        ros::service::call("follow_me/set_parameters",srv_req_,srv_resp_);
        ROS_INFO("Costmap deflated and DWA params set to move backwards");

        ros::Duration(1.0).sleep();
    };

    void resetParams()
    {
        dynamic_reconfigure::DoubleParameter param_double;
        dynamic_reconfigure::IntParameter param_int;
        dynamic_reconfigure::Config conf;
        param_int.name = "costmap_gradient";
        param_int.value = initial_global_costmap_gradient_;  
        conf.ints.push_back(param_int);
        srv_req_.config = conf;
        ros::service::call("A_star_local_planner/set_parameters",srv_req_,srv_resp_);
        ROS_INFO("A* Costmap reset to default");

        param_double.name = "max_speed";
        param_double.value = initial_max_speed_;
        conf.doubles.push_back(param_double);
        param_double.name = "min_speed";
        param_double.value = 0.0;  
        conf.doubles.push_back(param_double);
        param_int.name = "costmap_gradient";
        param_int.value = initial_local_costmap_gradient_;  
        conf.ints.back()= param_int;
        srv_req_.config = conf;
        ros::service::call("follow_me/set_parameters",srv_req_,srv_resp_);
        ROS_INFO("DWA and Costmap params reset to default");

        ros::Duration(1.0).sleep();

    };
    
    void setExpectedResult(BT::NodeStatus res)
    {

    expected_result_ = res;
    
    };    

    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<int>("follow_status") };
    };

  private:
    // using atomic because these variables might be accessed from different threads
// using atomic because these variables might be accessed from different threads
    BT::Duration time_;
    std::atomic<BT::NodeStatus> expected_result_;
    std::atomic<int> tick_count_;
    std::shared_ptr<ros::NodeHandle> nh_;
    ros::Publisher goal_pub_;
    ros::Subscriber status_sub_;
    dynamic_reconfigure::ReconfigureRequest srv_req_;
    dynamic_reconfigure::ReconfigureResponse srv_resp_;

    // dynamic_reconfigure::IntParameter * param_ptr_;
    
    double initial_max_speed_;
    int initial_local_costmap_gradient_;
    int initial_global_costmap_gradient_;
    geometry_msgs::PoseStamped goal_;
    std_msgs::Int32 status_;
    double last_published_ = 0;
    double last_received_status_ = 0;
    int count_ = 0;
    int reverse_count_ = 40;

};
};

#endif
