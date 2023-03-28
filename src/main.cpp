#include <iostream>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include "check_end_reached.h"
#include "Ros.h"

using namespace std::chrono_literals;

//NODES AS FUNCTIONS
BT::NodeStatus CrawlForward(){
    std::cout << "Crawling Forward" << std::endl;
    std::this_thread::sleep_for(2s);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TetherTaught(){
    std::cout << "Waiting til tether is taught" << std::endl;
    int reached_end;
    std::cin >> reached_end;
    if (std::cin.fail())
        throw "Sorry, I don't think that's a number?";
    if (reached_end < 0)
        throw "Sorry, no negative numbers. Try something else? ";

    if (reached_end){
        std::cout << "Tether Taught!" << std::endl;
        return BT::NodeStatus::SUCCESS;
    } else {
        std::cout << "Tether Not Taught" << std::endl;
        return BT::NodeStatus::RUNNING;
    }    
}

BT::NodeStatus TakeMeasurement(){
    std::cout << "Waiting til tether is taught" << std::endl;
    int reached_end;
    std::cin >> reached_end;
    if (std::cin.fail())
        throw "Sorry, I don't think that's a number?";
    if (reached_end < 0)
        throw "Sorry, no negative numbers. Try something else? ";

    if (reached_end){
        std::cout << "Tether Taught!" << std::endl;
        return BT::NodeStatus::SUCCESS;
    } else {
        std::cout << "Tether Not Taught" << std::endl;
        return BT::NodeStatus::RUNNING;
    }    
}

//NODES AS MEMBER FUNCTIONS
class Tether{
    public:
    Tether(){
    }
    BT::NodeStatus pullTether(){
        std::cout << "Pulling Reel "<< std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus stopTether(){
        std::cout << "Stopping Reel" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

int main(int argc, char** argv){
    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<CheckEndReached>("CheckEndReached");
    factory.registerSimpleCondition("CrawlForward", std::bind(CrawlForward));
    factory.registerSimpleCondition("TetherTaught", std::bind(TetherTaught));
    Tether tether;

    factory.registerSimpleAction(
        "PullTether",
        std::bind(&Tether::pullTether, &tether));

    factory.registerSimpleAction(
        "StopTether",
        std::bind(&Tether::stopTether, &tether));

    auto tree = factory.createTreeFromFile("./../bt_tree.xml");

    // tree.tickOnce();
    Ros Ros(argc, argv, "behavior_tree");
    tree.tickWhileRunning();
    std::cout << "Inspection successful" << std::endl;
    return 0;
}