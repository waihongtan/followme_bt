#include <followme_bt/action_plan_backwards_node_ros.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>

namespace follow_me_bt_action_nodes
{

BT::NodeStatus SyncActionPlanBackwards::tick()
{
    // using std::chrono::high_resolution_clock;
    this->setExpectedResult(BT::NodeStatus::FAILURE);
    tick_count_++;
    ros::Rate r_(15);
    this->getInitialParams();
    this->setBackwardsParams();
    last_published_ = ros::Time::now().toSec();
    count_ = 0;
    while (ros::Time::now().toSec() - last_published_ <= 10.0 && ros::ok() && count_ <= reverse_count_)
    {
        goal_.header.stamp =  ros::Time::now() ;
        goal_pub_.publish(goal_);
        r_.sleep();
        if ((ros::Time::now().toSec() - last_received_status_ <= 0.2))
        {
            if (status_.data == 1)
            {
            ROS_INFO("Move backwards successful for %d times",count_);
            count_ += 1;
            }
        }

        else
        {
            ROS_WARN("No feedback received,still trying to move backwards");
        }
    
    }
    if (count_ >= reverse_count_)
    {
        if( !setOutput("follow_status", status_.data) )
        {
            throw BT::RuntimeError("Plan Backwards failed output");
        }
        SyncActionPlanBackwards::setExpectedResult(BT::NodeStatus::SUCCESS);
    }
    else
    {
        if( !setOutput("follow_status", 0) )
        {
            throw BT::RuntimeError("Plan Backwards failed output");
        }
        ROS_WARN("Move backwards failed, trying next recovery!");
        SyncActionPlanBackwards::setExpectedResult(BT::NodeStatus::FAILURE);

    }
    this->resetParams();
    return expected_result_;
};

};


