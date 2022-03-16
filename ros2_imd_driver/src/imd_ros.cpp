#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ros2_imd_interfaces/msg/motor_cmd.hpp>
#include <ros2_imd_interfaces/msg/motor_feed.hpp>

#include "imd_ros.hpp"

IMDNode::IMDNode(const std::string &name_space, const rclcpp::NodeOptions &options) :
    Node("imd_node", name_space, options), tf_broadcaster_(this)
{
    RCLCPP_INFO(this->get_logger(), "Starting ns=%s,exec=%s", this->get_namespace(), this->get_name());
    if(strlen(this->get_namespace()) > 1)
    {
        frame_ns_ = std::string(this->get_namespace()).substr(1) + ".";
        std::replace(frame_ns_.begin(), frame_ns_.end(), '/', '.');
    }
    else
        frame_ns_ = std::string("");

    this->declare_parameter("publish_tf", true);

    this->declare_parameter("mcp2210_serial_number", "00000000000");
    this->declare_parameter("mcp2210_cs_pin", 0);

    for (auto &m : motor_)
    {
        this->declare_parameter(m.name + ".encoder_cpr", 0);
        this->declare_parameter(m.name + ".gear_ratio", 0.0);
        this->declare_parameter(m.name + ".max_rps", 0.0);
        this->declare_parameter(m.name + ".dir_reverse", false);
        this->declare_parameter(m.name + ctrl_name_.current + ctrl_name_.param.kp, 0.0);
        this->declare_parameter(m.name + ctrl_name_.current + ctrl_name_.param.ki, 0.0);
        this->declare_parameter(m.name + ctrl_name_.current + ctrl_name_.param.kd, 0.0);
        this->declare_parameter(m.name + ctrl_name_.velocity + ctrl_name_.param.kp, 0.0);
        this->declare_parameter(m.name + ctrl_name_.velocity + ctrl_name_.param.ki, 0.0);
        this->declare_parameter(m.name + ctrl_name_.velocity + ctrl_name_.param.kd, 0.0);
    }

    std::string param_serial_num;
    int param_cs_pin;
    std::wstring mcp2210_serial_number;
    MCP2210Linux::cs_pin_t mcp2210_cs_pin;

    publish_tf_ = this->get_parameter("publish_tf").as_bool();

    bool sn_was_set = this->has_parameter("mcp2210_serial_number");
    if(sn_was_set)
        param_serial_num = this->get_parameter("mcp2210_serial_number").as_string();
    if(param_serial_num[0] == 'i') //for launch argument string cast 
            param_serial_num = param_serial_num.substr(1);

    bool cs_was_set = this->has_parameter("mcp2210_cs_pin");
    if(cs_was_set)
        param_cs_pin = this->get_parameter("mcp2210_cs_pin").as_int();

    std::wstring sn;
    if (sn_was_set)
    {
        sn = std::wstring_convert<std::codecvt_utf8<wchar_t>>()
                 .from_bytes(param_serial_num);
    }

    auto dev = MCP2210Linux::get_dev_info();
    if (dev == NULL)
    {
        RCLCPP_FATAL(this->get_logger(), "MCP2210 was not found.");
        exit(-1);
    }
    RCLCPP_INFO(this->get_logger(), "MCP2210 SerialNumber List-------");
    RCLCPP_INFO(this->get_logger(), "File path\t: Serial number");

    while (dev != NULL)
    {
        RCLCPP_INFO(this->get_logger(), "%s\t: %ls", dev->path, dev->serial_number);

        if (sn.length() == 0)
            mcp2210_serial_number = std::wstring(dev->serial_number);
        else
        {
            if (sn == dev->serial_number)
                mcp2210_serial_number = sn;
        }
        dev = dev->next;
    }
    RCLCPP_INFO(this->get_logger(), "--------------------------------");

    if (mcp2210_serial_number.length() != 0)
        RCLCPP_INFO(this->get_logger(), "Use MCP2210 with serial number: %ls.", mcp2210_serial_number.c_str());
    else
        RCLCPP_ERROR(this->get_logger(), "The serial number of MCP2210 is invalid. %ls.\n", mcp2210_serial_number.c_str());

    if (cs_was_set)
    {
        mcp2210_cs_pin = (MCP2210Linux::cs_pin_t)param_cs_pin;

        if (mcp2210_cs_pin > MCP2210Linux::GP8)
            RCLCPP_WARN(this->get_logger(), "The chip selector pin name is incorrect. So we will use 0 (GP0) instead.\n");
        else
            RCLCPP_INFO(this->get_logger(), "GP%d was specified as the chip selector pin name.\n", mcp2210_cs_pin);
    }
    else
    {
        mcp2210_cs_pin = MCP2210Linux::GP0;

        RCLCPP_WARN(this->get_logger(), "The chip selector pin name is not specified. So we will use 0 (GP0) instead.\n");
    }

    
    md_.reset(new IMDController((wchar_t *)mcp2210_serial_number.c_str(), mcp2210_cs_pin));
    

    bool fatal_init_param = false;
    for (auto &m : motor_)
    {
        m.param.encoder_cpr = this->get_parameter(m.name + ".encoder_cpr").as_int();
        m.param.gear_ratio = this->get_parameter(m.name + ".gear_ratio").as_double();
        m.param.max_rps = this->get_parameter(m.name + ".max_rps").as_double();
        m.param.dir_reverse = this->get_parameter(m.name + ".dir_reverse").as_bool();

        if (m.param.encoder_cpr <= 0)
        {
            RCLCPP_FATAL(this->get_logger(), "'%s' is invalid value.\t(correct range : x > 0)", (m.name + ".encoder_cpr").c_str());
            fatal_init_param = true;
        }
        else
            RCLCPP_INFO(this->get_logger(), "%s : %d[count/revolution]", (m.name + ".encoder_cpr").c_str(), m.param.encoder_cpr);

        if (m.param.gear_ratio <= 0.0)
        {
            RCLCPP_FATAL(this->get_logger(), "'%s' is invalid value.\t(correct range : x > 0.0)", (m.name + ".gear_ratio").c_str());
            fatal_init_param = true;
        }
        else
            RCLCPP_INFO(this->get_logger(), "%s : %f[-]", (m.name + ".gear_ratio").c_str(), m.param.gear_ratio);

        if (m.param.max_rps <= 0.0)
        {
            RCLCPP_FATAL(this->get_logger(), "'%s' is invalid value.\t(correct range : x > 0.0)", (m.name + ".max_rps").c_str());
            fatal_init_param = true;
        }
        else
            RCLCPP_INFO(this->get_logger(), "%s : %f[revolution/second]", (m.name + ".max_rps").c_str(), m.param.max_rps);

        m.param.cur.kp = this->get_parameter(m.name + ctrl_name_.current + ctrl_name_.param.kp).as_double();
        m.param.cur.ki = this->get_parameter(m.name + ctrl_name_.current + ctrl_name_.param.ki).as_double();
        m.param.cur.kd = this->get_parameter(m.name + ctrl_name_.current + ctrl_name_.param.kd).as_double();

        m.param.vel.kp = this->get_parameter(m.name + ctrl_name_.velocity + ctrl_name_.param.kp).as_double();
        m.param.vel.ki = this->get_parameter(m.name + ctrl_name_.velocity + ctrl_name_.param.ki).as_double();
        m.param.vel.kd = this->get_parameter(m.name + ctrl_name_.velocity + ctrl_name_.param.kd).as_double();

        RCLCPP_INFO(this->get_logger(), "'%s' PID controller parameters[p,i,d] :\t[%f, %f, %1f]",
                    (m.name + ctrl_name_.current).c_str(), m.param.cur.kp, m.param.cur.ki, m.param.cur.kd);
        RCLCPP_INFO(this->get_logger(), "'%s' PID controller parameters[p,i,d] :\t[%f, %f, %f]\n",
                    (m.name + ctrl_name_.velocity).c_str(), m.param.vel.kp, m.param.vel.ki, m.param.vel.kd);
    }

    if (!fatal_init_param)
    {
        resetIMD_();

        process_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(
                &IMDNode::processCallback_,
                this));

        for (int i = 0; i < (int)motor_.size(); i++)
        {
            motor_[i].pub = this->create_publisher<MotorFeedMsg>(
                motor_[i].name + "/feedback",
                rclcpp::QoS(10));
            std::function<void(const MotorCmdMsg::SharedPtr msg)> sub_cb =
                std::bind(
                    &IMDNode::motorCmdCallback_,
                    this,
                    std::placeholders::_1, i);
            motor_[i].sub = this->create_subscription<MotorCmdMsg>(
                motor_[i].name + "/command",
                rclcpp::QoS(10),
                sub_cb);
        }

        reset_srv_ = this->create_service<ResetSrv>(
            "imd_reset",
            std::bind(
                &IMDNode::resetSrvCallback_,
                this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot start control because the parameters were invalid.(press ctrl+c to stop)");
    }
}

IMDNode::~IMDNode()
{
    process_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Destroyed ns=%s,exec=%s process.", this->get_namespace(), this->get_name());
}

void IMDNode::motorCmdCallback_(const MotorCmdMsg::SharedPtr msg, const int m_index)
{
    motor_[m_index].cmd_updated = true;
    motor_[m_index].cmd_vel = msg->velocity;
}

void IMDNode::publishTransform_(const MotorFeedMsg& msg,
                                const IMDNode::motor_node_t& m)
{
    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = msg.header.stamp;
    transform.header.frame_id = frame_ns_ + m.name + ".frame";
    transform.child_frame_id = frame_ns_ + m.name + ".link";
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = sin(msg.pose/2);
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = cos(msg.pose/2);

    tf_broadcaster_.sendTransform(transform);
}


void IMDNode::resetIMD_()
{
    std::lock_guard<std::mutex> lock(mutex_);

    RCLCPP_INFO(this->get_logger(), "Activating controller. Please wait a little while...\n");
        
    md_->ctrl_begin(new IMDController::motor_param_t[2]{motor_[0].param, motor_[1].param});
        
    RCLCPP_INFO(this->get_logger(), "Controller is running.");
}

void IMDNode::resetSrvCallback_(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ResetSrv::Request> request, std::shared_ptr<ResetSrv::Response> response)
{
    (void)request_header;
    (void)request;
    (void)response;
    this->resetIMD_();
}

void IMDNode::processCallback_()
{
    std::lock_guard<std::mutex> lock(mutex_);

    for (int i = 0; i < (int)motor_.size(); i++)
    {
        if (motor_[i].cmd_updated)
        {
            float rps = motor_[i].cmd_vel / (2 * M_PI);
            md_->set_rps((IMDController::motor_t)i, rps);
        }
    }
    md_->update();

    if (md_->state_updated())
    {
        ctrl_feed_msg_t feed = md_->get_state();

        for (int i = 0; i < (int)motor_.size(); i++)
        {
            MotorFeedMsg feed_msg;
            feed_msg.header.stamp = this->get_clock()->now();
            feed_msg.header.frame_id = frame_ns_ + motor_[i].name + ".frame";
            feed_msg.pose = 2 * M_PI * feed.angle[i];
            feed_msg.velocity = 2 * M_PI * feed.velocity[i];
            feed_msg.velocity_error = feed_msg.velocity - motor_[i].cmd_vel;
            motor_[i].pub->publish(feed_msg);

            if(publish_tf_)
                publishTransform_(feed_msg, motor_[i]);
        }
    }
}
