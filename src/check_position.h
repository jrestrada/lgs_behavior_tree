#ifndef CHECKCRAWLERPOSITION_H
#define CHECKCRAWLERPOSITION_H

#include "behaviortree_cpp/action_node.h"
#include <chrono>

class OutputCrawlerPosition : public BT::SyncActionNode{
    public:
    explicit OutputCrawlerPosition(const std::string &name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config){
    }
    static BT::PortsList providedPorts(){
        return { BT::OutputPort<float>("feet_deployed")};
    }
    BT::NodeStatus tick() override{
        setOutput("feet_deployed",14);
        return BT::NodeStatus::SUCCESS;
    }
};

#endif // CheckCrawlerPosition_H