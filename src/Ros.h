#ifndef NODE_H
#define NODE_H

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "behaviortree_cpp/action_node.h"

#define LOG(...) RCLCPP_INFO(rclcpp::get_logger("behavior tree"), __VA_ARGS__)

class Ros {
public:
    static Ros *instance(void) { return s_self; }
    static void quit(void) { s_self->shutdown(); }
    Ros(int argc, char *argv[], const std::string &node_name);
    ~Ros();
    // Execute ROS
    void spin(void);
    void spinOnBackground(void);
    void shutdown(void);
    void publishBtStatus(std::string status);
    void publishActuationRequest(std::string request);
    int feetDeployed() {return m_ft_deployed;} ;
    bool Overriden() { return m_overriding;} ;
    BT::NodeStatus FrontIsConnected();
    BT::NodeStatus IsHorizontal();
    BT::NodeStatus IsVertical();
    BT::NodeStatus TetherIsTaught();
    BT::NodeStatus ReachedEnd();
    rclcpp::Node::SharedPtr node(void) { return m_node; }
    std::string current_cmd() {return m_cmd;};
    std::string current_log() {return m_log;};
    void set_command(std::string command);
    void set_log(std::string log);
    bool m_continue = false;
protected:
private:
    float m_front_accel = 0.0;
    float m_front_pitch = 0.0;
    float m_ft_to_end = 100.0;
    int m_ft_deployed = 0;
    bool m_probing_imu = false;
    bool m_overriding = true;
    std::string m_cmd;
    std::string m_log;
    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void LidarCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void TetherCallback(const std_msgs::msg::Int16::SharedPtr msg);
    void OverrideCallback(const std_msgs::msg::String::SharedPtr msg);
    void HeartbeatCallback(const std_msgs::msg::Bool::SharedPtr msg);
    rclcpp::Node::SharedPtr m_node;
    rclcpp::executors::StaticSingleThreadedExecutor::SharedPtr m_executor;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_actuation_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_bt_status_pub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_lidar_sub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr m_tether_sub;    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_heart_sub;    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_override_sub;    
    // Instance
    static Ros *s_self;
};

#endif // NODE_H
