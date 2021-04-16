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

#include "followme_bt/condition_is_oscillating_node_ros.h"

BT::ConditionIsOscillatingNode::ConditionIsOscillatingNode(const std::string& name,  const BT::NodeConfiguration & config, ros::NodeHandle* nh)
    : ConditionNode(name, config) 
{
    expected_result_ = NodeStatus::FAILURE;
    nh_ptr_ = *nh;
    tick_count_ = 0;
};

BT::NodeStatus BT::ConditionIsOscillatingNode::tick()
{
    getInput<int>("follow_status",follow_status_);
    if (follow_status_ == 2) {
        BT::ConditionIsOscillatingNode::setExpectedResult(NodeStatus::SUCCESS);
    }
return expected_result_;
};

void BT::ConditionIsOscillatingNode::setExpectedResult(NodeStatus res)
{
    expected_result_ = res;
};

