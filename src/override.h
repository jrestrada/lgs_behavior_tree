#ifndef OVERRIDE_H
#define OVERRIDE_H

#include "behaviortree_cpp/action_node.h"
#include "Ros.h"
#include "iostream"

class Override : public BT::ActionNodeBase{
    public:
    explicit Override(const std::string &name, const BT::NodeConfig& config) : BT::ActionNodeBase(name, config){
    }

    static BT::PortsList providedPorts(){
        return { BT::InputPort<int>("Input")};
    }
    BT::NodeStatus tick() override{
        
        if (Ros::instance()->Overriden()){
            Ros::instance()->m_continue = true;
            return BT::NodeStatus::RUNNING;
        } else {
            return BT::NodeStatus::SUCCESS;
        }
        // return Ros::instance()->Overriden() ?  BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }
    void halt() override{}
    private: 
};

#endif // OVERRIDE_H