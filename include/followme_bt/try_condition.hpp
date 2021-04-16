#ifndef CONDITIONTEST_HPP
#define CONDITIONTEST_HPP

#include "behaviortree_cpp_v3/condition_node.h"

namespace BT
{
class IsTrackedNode : public ConditionNode
{
  public:
    
    IsTrackedNode();

    // The method that is going to be executed by the thread
    virtual BT::NodeStatus tick() override;

    int tickCount() const
    {
        return tick_count_;
    }

  private:
    NodeStatus expected_result_;
    int tick_count_;
};
}

#endif
