#ifndef REQUESTACTUATION_H
#define REQUESTACTUATION_H

#include "behaviortree_cpp/action_node.h"
#include "Ros.h"

class RequestActuation : public BT::ActionNodeBase{
    public:
    explicit RequestActuation(const std::string &name, const BT::NodeConfig& config) : BT::ActionNodeBase(name, config){
        first_tick = true;
        continuous = false;
    }

    static BT::PortsList providedPorts(){
        return { BT::InputPort<int>("Input")};
    }
    BT::NodeStatus tick() override{
        if (first_tick){
            if (name().find("&") != std::string::npos){
                continuous = true;
            }
            first_tick = false;
        Ros::instance()->publishActuationRequest(name());
        }
        // if (!continuous) Ros::instance()->publishActuationRequest(name());
        return continuous ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
    }

    void halt() override{}
    private: 
    bool first_tick;
    bool continuous;
};

#endif // REQUESTACTUATION_H