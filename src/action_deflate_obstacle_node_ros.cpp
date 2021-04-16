/* Copyright (C) 2015-2017 Michele Colledanchise -  All Rights Reserved
 * Copyright (C) 2018-2019 Davide Faconti, Eurecat -  All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "followme_bt/action_deflate_obstacle_node_ros.h"
#include <string>
#include <geometry_msgs/PoseStamped.h>

BT::AsyncActionDeflateObstacle::AsyncActionDeflateObstacle(const std::string& name,  const NodeConfiguration & config, ros::NodeHandle* nh , BT::Duration deadline_ms) :
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
    goal_sub_ = nh_ptr_.subscribe("/global_goal", 1, &AsyncActionDeflateObstacle::callback,this);

}

BT::NodeStatus BT::AsyncActionDeflateObstacle::tick()
{
    using std::chrono::high_resolution_clock;
    tick_count_++;
    boost::shared_ptr<std_msgs::Int32 const> status_ptr;
    

    auto initial_time = high_resolution_clock::now();
    nh_ptr_.param<int>("follow_me/costmap_gradient", initial_local_gradient_, 6.0);
    nh_ptr_.param<int>("A_star_local_planner/costmap_gradient", initial_global_gradient_, 6.0);
    ROS_INFO("Deflating inflation");
    param_ptr_ = &param_;
    conf_.ints.push_back(* param_ptr_);
    param_.name = "costmap_gradient";
    param_.value = initial_local_gradient_ + 2;
    srv_req_.config = conf_;
    ros::service::call("follow_me/set_parameters",srv_req_,srv_resp_);
    param_.name = "costmap_gradient";
    param_.value = initial_global_gradient_ + 2;
    srv_req_.config = conf_;
    ros::service::call("A_star_local_planner/set_parameters",srv_req_,srv_resp_);
    // we simulate an asynchronous action that takes an amount of time equal to time_
    while ((!isHaltRequested() && high_resolution_clock::now() < initial_time + time_) || !(status_.data) == 1  )
    {
        goal_pub_.publish(goal_);
        status_ptr = ros::topic::waitForMessage<std_msgs::Int32>("/follow_status", nh_ptr_, ros::Duration(0.2));
        if (status_ptr != NULL){
        status_ = *status_ptr;
        }
        else {
            if( !setOutput("global_costmap_gradient", initial_global_gradient_) &&  !setOutput("local_costmap_gradient", initial_local_gradient_))
        {
            throw RuntimeError("IsTrackedNode failed output");
        }
        }
    }

    if ( status_.data == 1 ){
        BT::AsyncActionDeflateObstacle::setExpectedResult(NodeStatus::SUCCESS);
        BT::AsyncActionDeflateObstacle::resetParameters();

    }
    // if( !setOutput("follow_status", status_.data) )
    //     {
    //         throw RuntimeError("IsTrackedNode failed output");
    //     }

    // check if we exited the while(9 loop because of the flag stop_loop_
    if( isHaltRequested() ){
        return NodeStatus::IDLE;
        BT::AsyncActionDeflateObstacle::resetParameters();
    }

    if( expected_result_ == NodeStatus::SUCCESS){
        success_count_++;
    }
    else if( expected_result_ == NodeStatus::FAILURE){
        failure_count_++;
    }

    return expected_result_;
};

void BT::AsyncActionDeflateObstacle::resetParameters(){
    ROS_INFO("Setting inflation back to default params");
    conf_.ints.push_back(* param_ptr_);
    param_.name = "costmap_gradient";
    param_.value = initial_local_gradient_ ;
    srv_req_.config = conf_;
    ros::service::call("follow_me/set_parameters",srv_req_,srv_resp_);
    param_.name = "costmap_gradient";
    param_.value = initial_global_gradient_;
    srv_req_.config = conf_;
    ros::service::call("A_star_local_planner/set_parameters",srv_req_,srv_resp_);
}

void BT::AsyncActionDeflateObstacle::halt()
{
    // do more cleanup here if necessary
    AsyncActionNode::halt();
}

void BT::AsyncActionDeflateObstacle::setTime(BT::Duration time)
{
    time_ = time;
}

void BT::AsyncActionDeflateObstacle::setExpectedResult(BT::NodeStatus res)
{
    expected_result_ = res;
}

void BT::AsyncActionDeflateObstacle::callback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg)
{
    goal_ = * goal_msg;
}
