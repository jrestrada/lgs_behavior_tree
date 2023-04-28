#include <iostream>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include "check_position.h"
#include "request_actuation.h"
#include "log_status.h"
#include "Ros.h"

using namespace std::chrono_literals;
std::string xml_path = "/home/josue/ros2_ws/src/lgs_bt/bt_tree.xml";

//NODES AS FUNCTIONS
BT::NodeStatus CrawlForward(){
    std::cout << "Crawling Forward" << std::endl;
    std::this_thread::sleep_for(2s);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RunningWhileFalse(){
    std::cout << "enter: 1 for success, 0 for running, any other for failure" << std::endl;
    int input;
    std::cin >> input;
    if (std::cin.fail())
        throw "Sorry, only numbers Allowed ";
    if (input < 0)
        throw "Sorry, no negative numbers. Try something else? ";
    if (input == 1){
        std::cout << "SUCCESS!" << std::endl;
        return BT::NodeStatus::SUCCESS;
    } else if (input == 0) {
        std::cout << "RUNNING" << std::endl;
        return BT::NodeStatus::RUNNING;
    } else {
        std::cout << "FAILURE" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}

int main(int argc, char** argv){
    BT::BehaviorTreeFactory factory;
    Ros ros(argc, argv, "behavior_tree");
    factory.registerNodeType<OutputCrawlerPosition>("OutputCrawlerPosition");
    factory.registerNodeType<RequestActuation>("RequestActuation");
    factory.registerNodeType<PublishBtStatus>("LogThisBranchName");
    factory.registerSimpleCondition("CheckEndReached", std::bind(&Ros::ReachedEnd, &ros));
    factory.registerSimpleCondition("FrontIsConnected", std::bind(&Ros::FrontIsConnected, &ros));
    // factory.registerSimpleCondition("CheckEndReached",std::bind(RunningWhileFalse));    // Console Version
    factory.registerSimpleCondition("IsVertical", std::bind(&Ros::IsVertical, &ros));
    factory.registerSimpleCondition("IsHorizontal", std::bind(&Ros::IsHorizontal, &ros));
    factory.registerSimpleCondition("TetherTaught", std::bind(&Ros::TetherIsTaught, &ros));
    // factory.registerSimpleCondition("TetherTaught", std::bind(RunningWhileFalse)); // Console Version
    // auto tree = factory.createTreeFromFile("./../bt_tree.xml");
    auto tree = factory.createTreeFromFile(xml_path);
    ros.spinOnBackground();
    tree.tickWhileRunning();
    std::cout << "Inspection successful" << std::endl;
    return 0;
}