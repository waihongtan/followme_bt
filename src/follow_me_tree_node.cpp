#include <followme_bt/action_plan_normally_node_ros.h>
#include <followme_bt/condition_is_tracked_node_ros.h>
#include <ros/ros.h>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

static const char* main_xml = R"(
    <root main_tree_to_execute = "MainTree">
        <BehaviorTree ID="MainTree">
            <Sequence name="RootSequence">
                <IsTracked name="is_tracked"/>
                <Fallback>
                    <PlanNormally name="plan_to_goal"/>
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

    BT::BehaviorTreeFactory factory;
    factory.registerBuilder<follow_me_bt_condition_nodes::ConditionIsTrackedNode>("IsTracked", is_tracked_condition_node_builder);
    factory.registerBuilder<follow_me_bt_action_nodes::SyncActionPlanNormally>("PlanNormally", plan_normally_action_node_builder);

    auto tree = factory.createTreeFromText(main_xml);
    
     while(true){
        ros::spinOnce();
        
        while(tree.tickRoot() == BT::NodeStatus::RUNNING)
        {
            std::this_thread::sleep_for( std::chrono::milliseconds(10) );
        }
        tree.haltTree();
    }

}
