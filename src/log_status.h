#ifndef LOG_BT_STATUS_H
#define LOG_BT_STATUS_H

#include "behaviortree_cpp/action_node.h"
#include "Ros.h"

class PublishBtStatus : public BT::SyncActionNode{
    public:
    explicit PublishBtStatus(const std::string &name) : BT::SyncActionNode(name, {}){
    }

    BT::NodeStatus tick() override{
        Ros::instance()->publishBtStatus(name());
        return BT::NodeStatus::SUCCESS;
    }
};

#endif // LOG_BT_STATUS_H