#ifndef NODE_H
#define NODE_H

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int16.hpp>

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

    rclcpp::Node::SharedPtr node(void) { return m_node; }
protected:
private:
    rclcpp::Node::SharedPtr m_node;
    rclcpp::executors::StaticSingleThreadedExecutor::SharedPtr m_executor;
    // Publishers, subscribers and servers
    std::string m_window;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr m_publisher;
    std_msgs::msg::Int16 m_feet_deployed;
    // Instance
    static Ros *s_self;
};

#endif // NODE_H
