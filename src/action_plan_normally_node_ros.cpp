#include <followme_bt/action_plan_normally_node_ros.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>

namespace follow_me_bt_action_nodes
{

BT::NodeStatus SyncActionPlanNormally::tick()
{
    // using std::chrono::high_resolution_clock;
    tick_count_++;
    ros::Rate r_(15);
    this->setExpectedResult(BT::NodeStatus::FAILURE);
    stucked_timer = ros::Time::now().toSec();
    stucked_duration = 0;

    while (ros::Time::now().toSec() - last_received_goal_ <= 0.1 && ros::ok())
    {
        goal_.header.stamp =  ros::Time::now() ;
        goal_pub_.publish(goal_);
        r_.sleep();
        if ((ros::Time::now().toSec() - last_received_status_ <= 0.2))
        {
            if (status_.data == 1 || status_.data == 3)
            {
                ROS_INFO("Planning successful");
                if (status_.data == 1)
                {
                    stucked_timer = ros::Time::now().toSec();
                }        
            }

            else if (status_.data == 0 && stucked_duration <= allowed_stucked_duration)
            {
                ROS_WARN("Robot is stucked for %f seconds",stucked_duration);
                stucked_duration = ros::Time::now().toSec() - stucked_timer;
            }

            else
            {
                if( !setOutput("follow_status", status_.data) )
                {
                    throw BT::RuntimeError("IsTrackedNode failed output");
                }
                ROS_WARN("Planning failed with status %d triggering recovery!",status_.data);
                SyncActionPlanNormally::setExpectedResult(BT::NodeStatus::FAILURE);
                return expected_result_;
            }
        }
        else
        {
            if( !setOutput("follow_status", 4) )
            {
                throw BT::RuntimeError("IsTrackedNode failed output");
            }
            ROS_WARN("No feedback received, planner might be down!");
            SyncActionPlanNormally::setExpectedResult(BT::NodeStatus::FAILURE);
            return expected_result_;
        }
    
    }
    SyncActionPlanNormally::setExpectedResult(BT::NodeStatus::SUCCESS);

    return expected_result_;
};

};


