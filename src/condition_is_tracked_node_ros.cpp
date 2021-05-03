/* Copyright (C) 2015-2017 Michele Colledanchise - All Rights Reserved
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

#include "followme_bt/condition_is_tracked_node_ros.h"

namespace follow_me_bt_condition_nodes{
BT::NodeStatus ConditionIsTrackedNode::tick()
{
    boost::shared_ptr<geometry_msgs::PoseStamped const> goal_ptr;
    geometry_msgs::PoseStamped goal;
    goal_ptr = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/leg_pose", *nh_, ros::Duration(0.2));
    if(goal_ptr != NULL){
        ROS_INFO("Tracking check done.");

        goal = *goal_ptr;
         if( !setOutput("output_goal", goal) )
        {
            throw BT::RuntimeError("IsTrackedNode failed output");
        }
        // setOutput("output_goal", goal);
        ConditionIsTrackedNode::setExpectedResult(BT::NodeStatus::SUCCESS);
    }
    return expected_result_;
};

void ConditionIsTrackedNode::setExpectedResult(BT::NodeStatus res)
{
    expected_result_ = res;
};
};
