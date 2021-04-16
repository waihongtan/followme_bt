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

#include "followme_bt/action_test_node_ros.h"
#include "followme_bt/condition_test_node_ros.h"
#include "followme_bt/action_deflate_obstacle_node_ros.h"
#include "followme_bt/condition_is_oscillating_node_ros.h"
#include "followme_bt/action_deflate_obstacle_node_ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "followme_bt/action_move_backwards_node_ros.h"

using BT::NodeStatus;
using std::chrono::milliseconds;



int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_behavior_tree");
    ros::NodeHandle nh; 
    using namespace BT;

    // BehaviorTreeFactory factory;
    // factory.registerNodeType<CalculateGoal>("CalculateGoal");
    // factory.registerNodeType<PrintTarget>("PrintTarget");

    // auto tree = factory.createTreeFromText(xml_text);
    NodeStatus status = NodeStatus::IDLE;

    auto bb = Blackboard::create();
    NodeConfiguration config;
    config.blackboard = bb;
    // config.input_ports["in_port"]   = "{my_input_port}";
    config.output_ports["output_goal"] = "{my_output_port}";
    config.input_ports["input_goal"] = "{my_output_port}";
    config.output_ports["follow_status"] = "{follow_status}";
    config.input_ports["follow_status"] = "{follow_status}";
    config.output_ports["global_costmap_gradient"] = "{global_costmap_gradient}";
    config.input_ports["global_costmap_gradient"] = "{global_costmap_gradient}";
    config.output_ports["local_costmap_gradient"] = "{local_costmap_gradient}";
    config.input_ports["local_costmap_gradient"] = "{local_costmap_gradient}";

    SequenceNode root("root_sequence");
    FallbackNode fallback_1("fallback1");
    FallbackNode fallback_2("fallback2");
    SequenceNode oscillating_sequence("oscillating_sequence");
    ConditionIsTrackedNode is_tracked_condition("is_tracked",config,&nh );
    AsyncActionPlanNormally plan_normally_action("plan_normally", config,&nh ,milliseconds(100));
    ConditionIsOscillatingNode is_oscillating_condition("is_oscillating",config,&nh);
    AsyncActionDeflateObstacle deflate_obstacle_action("deflate_obstacle", config , &nh,milliseconds(5000));
    AsyncActionMoveBackwards move_backwards_action("move_backwards", config , &nh,milliseconds(5000));
    root.addChild(&is_tracked_condition);
    root.addChild(&fallback_1);
    fallback_1.addChild(&plan_normally_action);
    fallback_1.addChild(&fallback_2);
    fallback_2.addChild(&oscillating_sequence);
    oscillating_sequence.addChild(&is_oscillating_condition);
    oscillating_sequence.addChild(&deflate_obstacle_action);
    oscillating_sequence.addChild(&move_backwards_action);
    while( ros::ok() && (status == NodeStatus::IDLE || status == NodeStatus::RUNNING))
  {
    ros::spinOnce();
    status = root.executeTick();
    // std::cout <<  bb->get<geometry_msgs::PoseStamped>("my_output_port") << std::endl;
    // std::cout << status << std::endl;
    // ros::Duration sleep_time(0.01);
    // sleep_time.sleep();
  }

  return 0;
};