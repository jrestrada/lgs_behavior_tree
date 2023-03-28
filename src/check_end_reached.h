#ifndef CheckEndReached_H
#define CheckEndReached_H

#include "behaviortree_cpp/action_node.h"
#include <chrono>

class CheckEndReached : public BT::SyncActionNode{
    public:
    explicit CheckEndReached(const std::string &name) : BT::SyncActionNode(name, {}){

    }

    BT::NodeStatus tick() override{
    int reached_end;
    std::cout << "Ticked:" << this->name() << " enter 1 if reached, 0 if not"<<std::endl;
    std::cin >> reached_end;
    if (std::cin.fail())
        throw "Sorry, I don't think that's a number?";
    if (reached_end < 0)
        throw "Sorry, no negative numbers. Try something else? ";

    if (reached_end){
        std::cout << " End Reached" << std::endl;
        return BT::NodeStatus::SUCCESS;
    } else {
        std::cout << " End Not Reached" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    }
};

#endif // CheckEndReached_H