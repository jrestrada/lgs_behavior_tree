#include "Ros.h"
#include <thread>
#include <signal.h>

void kill(int /*sig*/) {
}

Ros *Ros::s_self = nullptr;

Ros::Ros(int argc, char *argv[], const std::string &node_name) {
    // Initilize ROS
    rclcpp::init(argc, argv);
    // Create ROS executer and node
    m_executor = rclcpp::executors::StaticSingleThreadedExecutor::make_shared();
    m_node = rclcpp::Node::make_shared(node_name);
    m_executor->add_node(m_node);
    // Add ROS publisher and subscribers
    m_actuation_pub = m_node->create_publisher<std_msgs::msg::String>("lgs_actuation_requests", 10);
    m_bt_status_pub = m_node->create_publisher<std_msgs::msg::String>("bt_status", 10);
    m_imu_sub = m_node->create_subscription<sensor_msgs::msg::Imu>("Imu",10,std::bind(&Ros::ImuCallback,this,std::placeholders::_1));
    if (s_self) {
        LOG("Ops, only one instance of 'Ros' can be created!");
    }
    else {
        s_self = this; 
        LOG("Ros created...");
    }
    signal(SIGINT, kill);
}

Ros::~Ros() {
    rclcpp::shutdown(); 
    LOG("ROS shutdown!");
}

void Ros::spin(void) {
    m_executor->spin();
}
BT::NodeStatus Ros::pullTether(void) {
    auto msg = std_msgs::msg::String();
    msg.data = "backward";
    LOG("Publishing request for: %s", msg.data.c_str());
    m_actuation_pub->publish(msg);
    return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus Ros::stopReel(void) {
    auto msg = std_msgs::msg::String();
    msg.data = "stop";
    LOG("Publishing request for: %s", msg.data.c_str());
    m_actuation_pub->publish(msg);
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus Ros::CrawlForward(void) {
    auto msg = std_msgs::msg::String();
    msg.data = "forward";
    LOG("Publishing request for: %s", msg.data.c_str());
    m_actuation_pub->publish(msg);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Ros::TetherIsTaught(){
    m_probing_accel = true;
    if (m_front_accel) {
        LOG("Acceleration detected: %f", m_front_accel);
        m_front_accel = 0;
        m_probing_accel = false;
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::RUNNING;
    }
}

void Ros::spinOnBackground(void) {
    std::thread thread(std::bind(&rclcpp::executors::StaticSingleThreadedExecutor::spin, m_executor));
    thread.detach();
}

void Ros::publishBtStatus(std::string branch_name) {
    LOG("Behavior Tree Status %s", branch_name.c_str());
    auto msg = std_msgs::msg::String();
    msg.data=branch_name;
    m_bt_status_pub->publish(msg);
}

void Ros::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg){
    if (m_probing_accel)
        LOG("ImuMSG received");
        m_front_accel = msg->linear_acceleration.x;
}

void Ros::shutdown(void) {
    m_executor->cancel();
}
