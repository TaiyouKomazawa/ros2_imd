#include <chrono>
#include <codecvt>
#include <functional>
#include <locale>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <math.h>
#include <wchar.h>

#include <IMDController.hpp>

#include <rclcpp/rclcpp.hpp>
#include <ros2_imd_interfaces/msg/motor_cmd.hpp>
#include <ros2_imd_interfaces/msg/motor_feed.hpp>

#include <tf2_ros/transform_broadcaster.h>

class IMDNode : public rclcpp::Node
{
    typedef ros2_imd_interfaces::msg::MotorFeed  MotorFeedMsg;
    typedef ros2_imd_interfaces::msg::MotorCmd   MotorCmdMsg;

    typedef rclcpp::Publisher<MotorFeedMsg>::SharedPtr MotorFeedPublisher;
    typedef rclcpp::Subscription<MotorCmdMsg>::SharedPtr MotorCmdSubscription;

public:
    IMDNode(
        const std::string &name_space,
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    IMDNode(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions()):IMDNode("", options) {}

    virtual ~IMDNode();

private:
    typedef struct MotorNodeType
    {
        const std::string name;
        IMDController::motor_param_t param;
        MotorFeedPublisher pub;
        MotorCmdSubscription sub;
        float cmd_vel;
        bool cmd_updated;
    } motor_node_t;

    std::vector<motor_node_t> motor_ = {
        {"m1",
         IMDController::motor_param_t(),
         MotorFeedPublisher(),
         MotorCmdSubscription(),
         0.0, false},
        {"m2",
         IMDController::motor_param_t(),
         MotorFeedPublisher(),
         MotorCmdSubscription(),
         0.0, false}
    };

    const struct
    {
        std::string current = ".pid.current";
        std::string velocity = ".pid.velocity";
        struct
        {
            std::string kp = ".kp";
            std::string ki = ".ki";
            std::string kd = ".kd";
        } param;
    } ctrl_name_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;
    std::string frame_ns_;

    std::shared_ptr<IMDController> md_;

    rclcpp::TimerBase::SharedPtr process_timer_;

    bool publish_tf_;

    void motorCmdCallback_(const MotorCmdMsg::SharedPtr msg, const int m_index);

    void publishTransform_(const MotorFeedMsg& msg, const motor_node_t& m);

    void processCallback_();
};
