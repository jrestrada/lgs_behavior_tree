#ifndef EVALUATEPOSITION_H
#define EVALUATEPOSITION_H

#include "behaviortree_cpp/action_node.h"
#include "Ros.h"
#include "iostream"

class EvaluatePosition : public BT::ActionNodeBase{
    public:
    explicit EvaluatePosition(const std::string &name, const BT::NodeConfig& config) : BT::ActionNodeBase(name, config){
    }

    static BT::PortsList providedPorts(){
        return { BT::InputPort<int>("Input")};
    }
    BT::NodeStatus tick() override{
        std::string sign = name().substr(0,1);
        std::string number = name().substr(1, *name().end() - 1);
        int target = stoi(number);
        int feet_deployed = Ros::instance()->feetDeployed();
        if (sign == ">"){
            if (feet_deployed > target){
                return BT::NodeStatus::SUCCESS;
            } else {
                return BT::NodeStatus::FAILURE;
            }
        } else if ( sign == "<"){
            if (feet_deployed < target){
                return BT::NodeStatus::SUCCESS;
            } else {
                return BT::NodeStatus::FAILURE;
            }
        } else {
            LOG("Incorrect Node Name");
            return BT::NodeStatus::FAILURE;
        }
    }
    void halt() override{}
    private: 
};

#endif // EVALUATEPOSITION_H