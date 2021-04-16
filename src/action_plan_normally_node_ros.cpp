#include <followme_bt/action_plan_normally_node_ros.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>

namespace follow_me_bt_action_nodes{

BT::NodeStatus SyncActionPlanNormally::tick()
{
    // using std::chrono::high_resolution_clock;
    // tick_count_++;
    boost::shared_ptr<std_msgs::Int32 const> status_ptr;
    std_msgs::Int32 status;

    // auto initial_time = high_resolution_clock::now();
    geometry_msgs::PoseStamped goal; 
    getInput<geometry_msgs::PoseStamped>("input_goal",goal);
    goal_pub_.publish(goal);
    std::cout <<"publish"<<std::endl;        
    status_ptr = ros::topic::waitForMessage<std_msgs::Int32>("/follow_status", *nh_, ros::Duration(0.2));
    
    if (status_ptr != NULL)
    {
        status = *status_ptr;
    }

    else if( !setOutput("follow_status", 0) )
    {
        throw BT::RuntimeError("IsTrackedNode failed output");
    }

    if ( status.data == 1 )
    {
        SyncActionPlanNormally::setExpectedResult(BT::NodeStatus::SUCCESS);
    }

    if( !setOutput("follow_status", status.data) )
    {
        throw BT::RuntimeError("IsTrackedNode failed output");
    }

    return expected_result_;
};

};


