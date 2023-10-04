#ifndef LOG_BT_STATUS_H
#define LOG_BT_STATUS_H

#include "behaviortree_cpp/action_node.h"
#include "Ros.h"

class PublishBtStatus : public BT::SyncActionNode{
    public:
    explicit PublishBtStatus(const std::string &name) : BT::SyncActionNode(name, {}){
        first_tick = true;
    }

    BT::NodeStatus tick() override{
        if (first_tick){
            current_log = name();
            first_tick = false;
            Ros::instance()->publishBtStatus(name());
            if (name() != "update_log"){
                Ros::instance()->set_log(current_log);
            }
        } else {
            if (name() == "update_log" && Ros::instance()->m_continue){
                Ros::instance()->publishBtStatus(Ros::instance()->current_log());
                Ros::instance()->m_continue = false;
                return BT::NodeStatus::SUCCESS;
            }
        }
        return BT::NodeStatus::SUCCESS;
    }

    bool first_tick;
    std::string current_log;

};

#endif // LOG_BT_STATUS_H