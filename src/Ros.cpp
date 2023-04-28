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
    m_imu_sub = m_node->create_subscription<sensor_msgs::msg::Imu>("imu",10, std::bind(&Ros::ImuCallback,this,std::placeholders::_1));
    m_lidar_sub = m_node->create_subscription<std_msgs::msg::Float32>("lidar",10, std::bind(&Ros::LidarCallback,this,std::placeholders::_1));
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

void Ros::spinOnBackground(void) {
    std::thread thread(std::bind(&rclcpp::executors::StaticSingleThreadedExecutor::spin, m_executor));
    thread.detach();
}

void Ros::publishActuationRequest(std::string request) {
    LOG("Publishing request for: %s", request.c_str());
    auto msg = std_msgs::msg::String();
    msg.data = request;
    m_actuation_pub->publish(msg);
}

BT::NodeStatus Ros::TetherIsTaught(){
    m_probing_imu = true;
    if (abs(m_front_accel) > 1.0) {
        LOG("Acceleration detected: %f", m_front_accel);
        m_front_accel = 0;
        m_probing_imu = false;
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::RUNNING;
    }
}

BT::NodeStatus Ros::IsHorizontal(){
    LOG("Pitch angle: %f", m_front_pitch);
    if (abs(m_front_pitch) < 9.0) {
        LOG("Front module is horizontal");
        return BT::NodeStatus::SUCCESS;
    } else {
        LOG("Front module is not horizontal");
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus Ros::FrontIsConnected(){
    std::vector<rclcpp::TopicEndpointInfo> heartbeat_publishers = m_node->get_publishers_info_by_topic("front_heartbeat", false);
    if (heartbeat_publishers.size() == 0){
        LOG("Front module disconnected");
        return BT::NodeStatus::FAILURE;
    } else {
        LOG(heartbeat_publishers[0].node_name().c_str());
        return BT::NodeStatus::SUCCESS;
    }
}

BT::NodeStatus Ros::IsVertical(){
    LOG("Pitch angle: %f", m_front_pitch);
    if (abs(m_front_pitch) > 80) {
        LOG("Front module is vertical");
        return BT::NodeStatus::SUCCESS;
    } else {
        LOG("Front module is not vertical");
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus Ros::ReachedEnd(){
    LOG("Distance to end: %f", m_ft_to_end);
    if (m_ft_to_end < 1.0) {
        LOG("End reached");
        return BT::NodeStatus::SUCCESS;
    } else {
        LOG("End not reached");
        return BT::NodeStatus::RUNNING;
    }
}

void Ros::publishBtStatus(std::string branch_name) {
    LOG("Status %s", branch_name.c_str());
    auto msg = std_msgs::msg::String();
    msg.data = branch_name;
    m_bt_status_pub->publish(msg);
}

void Ros::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg){
    m_front_pitch = msg->orientation.y;
    if (m_probing_imu)
        m_front_accel = msg->linear_acceleration.x;
}

void Ros::LidarCallback(const std_msgs::msg::Float32::SharedPtr msg){
    m_ft_to_end = msg->data;
}

void Ros::shutdown(void) {
    m_executor->cancel();
}
