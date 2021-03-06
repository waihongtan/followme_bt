#include <followme_bt/action_plan_normally_node_ros.h>
#include <followme_bt/action_plan_backwards_node_ros.h>
#include <followme_bt/action_deflate_and_plan_backwards_node_ros.h>

#include <followme_bt/condition_is_tracked_node_ros.h>
#include <followme_bt/condition_is_oscillating_node_ros.h>
#include <followme_bt/condition_is_stucked_node_ros.h>


#include <ros/ros.h>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

static const char* main_xml = R"(
    <root main_tree_to_execute = "MainTree">
        <BehaviorTree ID="MainTree">
            <Sequence name="RootSequence">
                <IsTracked output_goal="{Goal}"/>
                <Fallback>
                    <PlanNormally name="plan_to_goal"  follow_status="{follow}"/>
                    <Fallback name="Recovery">
                        <Sequence>
                            <IsOscillating follow_status="{follow}"/>
                            <PlanBackwards name="plan_backwards"  follow_status="{follow}"/>
                        </Sequence>
                        <Sequence>
                            <IsStucked follow_status="{follow}"/>
                            <DeflateAndPlanBackwards name="deflate_and_plan_backwards"  follow_status="{follow__}"/>
                        </Sequence>
                    </Fallback>    
                </Fallback>
            </Sequence>
        </BehaviorTree>
    </root>
 )";



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "follow_me_bt");
    std::shared_ptr<ros::NodeHandle> shared_nh = std::make_shared<ros::NodeHandle>("~");
    BT::NodeBuilder is_tracked_condition_node_builder = [shared_nh](const std::string& name, const BT::NodeConfiguration& config ){
        return std::make_unique<follow_me_bt_condition_nodes::ConditionIsTrackedNode>(name,config,shared_nh);
    };

    BT::NodeBuilder plan_normally_action_node_builder = [shared_nh](const std::string& name, const BT::NodeConfiguration& config ){
        return std::make_unique<follow_me_bt_action_nodes::SyncActionPlanNormally>(name,config,shared_nh);
    };

    BT::NodeBuilder plan_backwards_action_node_builder = [shared_nh](const std::string& name, const BT::NodeConfiguration& config ){
        return std::make_unique<follow_me_bt_action_nodes::SyncActionPlanBackwards>(name,config,shared_nh);
    };

    BT::NodeBuilder deflate_and_plan_backwards_action_node_builder = [shared_nh](const std::string& name, const BT::NodeConfiguration& config ){
        return std::make_unique<follow_me_bt_action_nodes::SyncActionDeflateAndPlanBackwards>(name,config,shared_nh);
    };


    BT::BehaviorTreeFactory factory;
    factory.registerBuilder<follow_me_bt_condition_nodes::ConditionIsTrackedNode>("IsTracked", is_tracked_condition_node_builder);
    factory.registerBuilder<follow_me_bt_action_nodes::SyncActionPlanNormally>("PlanNormally", plan_normally_action_node_builder);
    factory.registerNodeType<follow_me_bt_condition_nodes::ConditionIsOscillatingNode>("IsOscillating");
    factory.registerBuilder<follow_me_bt_action_nodes::SyncActionPlanBackwards>("PlanBackwards", plan_backwards_action_node_builder);
    factory.registerNodeType<follow_me_bt_condition_nodes::ConditionIsStuckedNode>("IsStucked");
    factory.registerBuilder<follow_me_bt_action_nodes::SyncActionDeflateAndPlanBackwards>("DeflateAndPlanBackwards", deflate_and_plan_backwards_action_node_builder);


    BT::NodeStatus status = BT::NodeStatus::IDLE;
    auto tree = factory.createTreeFromText(main_xml);
    ros::AsyncSpinner spinner(1); 
    spinner.start();
    while(true && ros::ok() )
    {
        
        while((status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING) && ros::ok() )
        {
            status == tree.tickRoot();
            std::this_thread::sleep_for( std::chrono::milliseconds(50) );
        }
        tree.haltTree();
    }

}
