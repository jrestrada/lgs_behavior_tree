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
            if (continuous) {
                current_cmd = name().substr(1,*name().end()-1);
            } else {
                current_cmd = name();
            }
            Ros::instance()->publishActuationRequest(current_cmd);
            Ros::instance()->m_continue = false;
            if (name() != "continue"){
                Ros::instance()->set_command(current_cmd);
            }
        } else {
            if (name() == "continue" && Ros::instance()->m_continue){
                Ros::instance()->publishActuationRequest(Ros::instance()->current_cmd());
                // Ros::instance()->m_continue = false;
                return BT::NodeStatus::SUCCESS;
            }
        }
        return continuous ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
        // return BT::NodeStatus::SUCCESS;
    }
    void halt() override{}
    private: 
    bool first_tick;
    bool continuous;
    std::string current_cmd;
};

#endif // REQUESTACTUATION_H