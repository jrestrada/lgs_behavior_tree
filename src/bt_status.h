#ifndef BT_STATUS_PUBLISHER
#define BT_STATUS_PUBLISHER

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

#endif // BT_STATUS_PUBLISHER