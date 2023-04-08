#include <iostream>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include "check_position.h"
#include "bt_status.h"
#include "Ros.h"

using namespace std::chrono_literals;

//NODES AS FUNCTIONS
BT::NodeStatus CrawlForward(){
    std::cout << "Crawling Forward" << std::endl;
    std::this_thread::sleep_for(2s);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RunningWhileFalse(){
    std::cout << "RunningWhileFalse, input now" << std::endl;
    int reached_end;
    std::cin >> reached_end;
    if (std::cin.fail())
        throw "Sorry, I don't think that's a number?";
    if (reached_end < 0)
        throw "Sorry, no negative numbers. Try something else? ";

    if (reached_end){
        std::cout << "reached end!" << std::endl;
        return BT::NodeStatus::SUCCESS;
    } else {
        std::cout << "not reached end yet, running" << std::endl;
        return BT::NodeStatus::RUNNING;
    }    
}

int main(int argc, char** argv){
    BT::BehaviorTreeFactory factory;
    Ros ros(argc, argv, "behavior_tree");
    factory.registerNodeType<OutputCrawlerPosition>("OutputCrawlerPosition");
    factory.registerNodeType<PublishBtStatus>("LogThisBranchName"); // Change individual Branch Name in XML  
    factory.registerSimpleCondition("CheckEndReached",std::bind(RunningWhileFalse));
    // factory.registerSimpleCondition("TetherTaught", std::bind(&Ros::TetherIsTaught, &ros));
    factory.registerSimpleCondition("TetherTaught", std::bind(RunningWhileFalse));
    factory.registerSimpleAction("CrawlForward", std::bind(&Ros::CrawlForward, &ros));
    factory.registerSimpleAction("PullTether", std::bind(&Ros::pullTether, &ros));
    factory.registerSimpleAction("StopReel",std::bind(&Ros::stopReel, &ros));
    // auto tree = factory.createTreeFromFile("./../bt_tree.xml");
    auto tree = factory.createTreeFromFile("/home/josue/ros2_ws/src/lgs_bt/bt_tree.xml");
    ros.spinOnBackground();
    tree.tickWhileRunning();
    std::cout << "Inspection successful" << std::endl;
    return 0;
}