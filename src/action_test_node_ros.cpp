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

#include "followme_bt/action_test_node_ros.h"
#include <string>
#include <geometry_msgs/PoseStamped.h>

BT::AsyncActionPlanNormally::AsyncActionPlanNormally(const std::string& name,  const NodeConfiguration & config, ros::NodeHandle* nh , BT::Duration deadline_ms) :
    AsyncActionNode(name, config),
  success_count_(0),
  failure_count_(0)
{
    expected_result_ = NodeStatus::FAILURE;
    time_ = deadline_ms;
    tick_count_ = 0;
    nh_ptr_ = *nh;
    goal_pub_ = nh_ptr_.advertise<geometry_msgs::PoseStamped>("/global_goal_bt",1);

}

BT::NodeStatus BT::AsyncActionPlanNormally::tick()
{
    using std::chrono::high_resolution_clock;
    tick_count_++;
    boost::shared_ptr<std_msgs::Int32 const> status_ptr;
    std_msgs::Int32 status;

    auto initial_time = high_resolution_clock::now();
    geometry_msgs::PoseStamped goal; 
    getInput<geometry_msgs::PoseStamped>("input_goal",goal);
    goal_pub_.publish(goal);
    std::cout <<"publish"<<std::endl;
    // we simulate an asynchronous action that takes an amount of time equal to time_
    while (!isHaltRequested() && high_resolution_clock::now() < initial_time + time_ )
    {
        
        status_ptr = ros::topic::waitForMessage<std_msgs::Int32>("/follow_status", nh_ptr_, ros::Duration(0.2));
        if (status_ptr != NULL){
        status = *status_ptr;
        }
        else {
            if( !setOutput("follow_status", 0) )
            {
                throw RuntimeError("IsTrackedNode failed output");
            }
            return expected_result_;
        }
    }

    if ( status.data == 1 ){
        BT::AsyncActionPlanNormally::setExpectedResult(NodeStatus::SUCCESS);
    }
    if( !setOutput("follow_status", status.data) )
        {
            throw RuntimeError("IsTrackedNode failed output");
        }

    // check if we exited the while(9 loop because of the flag stop_loop_
    if( isHaltRequested() ){
        return NodeStatus::IDLE;
    }

    if( expected_result_ == NodeStatus::SUCCESS){
        success_count_++;
    }
    else if( expected_result_ == NodeStatus::FAILURE){
        failure_count_++;
    }

    return expected_result_;
}

void BT::AsyncActionPlanNormally::halt()
{
    // do more cleanup here if necessary
    AsyncActionNode::halt();
}

void BT::AsyncActionPlanNormally::setTime(BT::Duration time)
{
    time_ = time;
}

void BT::AsyncActionPlanNormally::setExpectedResult(BT::NodeStatus res)
{
    expected_result_ = res;
}
