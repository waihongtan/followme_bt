
#include "followme_bt/action_plan_backwards_node_ros.h"
#include <string>
#include <geometry_msgs/PoseStamped.h>

BT::AsyncActionMoveBackwards::AsyncActionMoveBackwards(const std::string& name,  const NodeConfiguration & config, ros::NodeHandle* nh , BT::Duration deadline_ms) :
    AsyncActionNode(name, config),
  success_count_(0),
  failure_count_(0)
{
    expected_result_ = NodeStatus::FAILURE;
    time_ = deadline_ms;
    tick_count_ = 0;
    nh_ptr_ = *nh;
    status_.data = 2;
    goal_pub_ = nh_ptr_.advertise<geometry_msgs::PoseStamped>("/global_goal_bt",1);
    getInput<int>("global_costmap_gradient",initial_global_gradient_);
    getInput<int>("local_costmap_gradient",initial_local_gradient_);
    ros::param::get("/follow_me/max_speed", initial_max_speed_);
    goal_.pose.position.x = -1.5;
    goal_.pose.position.y = 0.0;
    
    // goal_sub_ = nh_ptr_.subscribe("/global_goal", 1, &AsyncActionMoveBackwards::callback,this);

}

BT::NodeStatus BT::AsyncActionMoveBackwards::tick()
{
    using std::chrono::high_resolution_clock;
    tick_count_++;
    boost::shared_ptr<std_msgs::Int32 const> status_ptr;
    

    auto initial_time = high_resolution_clock::now();
    
    ROS_INFO("Planning backwards");
    // param_ptr_ = &param_;
    param_double_.name = "max_speed";
    param_double_.value = 0.0;
    conf_.doubles.push_back(param_double_);
    param_double_.name = "min_speed";
    param_double_.value = -0.5;  
    conf_.doubles.push_back(param_double_);
    srv_req_.config = conf_;
    ros::service::call("follow_me/set_parameters",srv_req_,srv_resp_);
    // we simulate an asynchronous action that takes an amount of time equal to time_
    while ((!isHaltRequested() && high_resolution_clock::now() < initial_time + time_) || !(status_.data) == 1  )
    {
       
        goal_.header.stamp = ros::Time::now();
        goal_pub_.publish(goal_);
        status_ptr = ros::topic::waitForMessage<std_msgs::Int32>("/follow_status", nh_ptr_, ros::Duration(0.2));
        if (status_ptr != NULL){
        status_ = *status_ptr;
        }
        // else {
        //     if( !setOutput("global_costmap_gradient", initial_global_gradient_) &&  !setOutput("local_costmap_gradient", initial_local_gradient_))
        // {
        //     throw RuntimeError("IsTrackedNode failed output");
        // }
        // }
    }

    if ( status_.data == 1 ){
        BT::AsyncActionMoveBackwards::setExpectedResult(NodeStatus::SUCCESS);
    }
    // if( !setOutput("follow_status", status_.data) )
    //     {
    //         throw RuntimeError("IsTrackedNode failed output");
    //     }

    // check if we exited the while(9 loop because of the flag stop_loop_
    if( isHaltRequested() ){
        return NodeStatus::IDLE;
        BT::AsyncActionMoveBackwards::resetParameters();
    }

    if( expected_result_ == NodeStatus::SUCCESS){
        success_count_++;
    }
    else if( expected_result_ == NodeStatus::FAILURE){
        failure_count_++;
    }
    BT::AsyncActionMoveBackwards::resetParameters();
    return expected_result_;
};

void BT::AsyncActionMoveBackwards::resetParameters(){
    ROS_INFO("Setting inflation and speed back to default params");
    //Reset local planner params 
    param_double_.name = "max_speed";
    param_double_.value = initial_max_speed_;
    conf_.doubles.push_back(param_double_);
    param_double_.name = "min_speed";
    param_double_.value = 0.0;  
    conf_.doubles.push_back(param_double_);
    param_.name = "costmap_gradient";
    param_.value = initial_local_gradient_;
    conf_.ints.push_back(param_);
    srv_req_.config = conf_;
    ros::service::call("follow_me/set_parameters",srv_req_,srv_resp_);
    //Reset A* planner params 
    conf_.ints.clear();
    conf_.doubles.clear();
    param_.name = "costmap_gradient";
    param_.value = initial_global_gradient_;
    conf_.ints.push_back(param_);
    srv_req_.config = conf_;
    ros::service::call("A_star_local_planner/set_parameters",srv_req_,srv_resp_);
}

void BT::AsyncActionMoveBackwards::halt()
{
    // do more cleanup here if necessary
    AsyncActionNode::halt();
}

void BT::AsyncActionMoveBackwards::setTime(BT::Duration time)
{
    time_ = time;
}

void BT::AsyncActionMoveBackwards::setExpectedResult(BT::NodeStatus res)
{
    expected_result_ = res;
}

// void BT::AsyncActionMoveBackwards::callback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg)
// {
//     goal_ = * goal_msg;
// }
