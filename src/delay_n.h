#ifndef DELAY_N_H
#define DELAY_N_H

#include "behaviortree_cpp/action_node.h"
#include "Ros.h"
#include <unistd.h>
#include "iostream"

class DelaySec : public BT::ActionNodeBase{
    public:
    explicit DelaySec(const std::string &name, const BT::NodeConfig& config) : BT::ActionNodeBase(name, config){
    }
    static BT::PortsList providedPorts(){
        return { BT::InputPort<int>("Input")};
    }
    BT::NodeStatus tick() override{
        std::string sign = name().substr(0,1);
        std::string number = name().substr(6, *name().end() - 1);
        int delay = stoi(number);
        LOG("Sleeping %i", delay);
        sleep(delay);
        return BT::NodeStatus::SUCCESS;
    }
    void halt() override{}
    private: 
};

#endif // DELAY_N_H