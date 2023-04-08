#ifndef NODE_H
#define NODE_H

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
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
    BT::NodeStatus pullTether();
    BT::NodeStatus stopReel();
    BT::NodeStatus CrawlForward();
    BT::NodeStatus TetherIsTaught();
    rclcpp::Node::SharedPtr node(void) { return m_node; }
protected:
private:
    float m_front_accel = 0;
    bool m_probing_accel = false;
    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    rclcpp::Node::SharedPtr m_node;
    rclcpp::executors::StaticSingleThreadedExecutor::SharedPtr m_executor;
    // Publishers, subscribers and servers
    std::string m_window;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_actuation_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_bt_status_pub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
    // Instance
    static Ros *s_self;
};

#endif // NODE_H
