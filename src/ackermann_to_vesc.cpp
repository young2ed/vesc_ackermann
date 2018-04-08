#include "vesc_ackermann/ackermann_to_vesc.h"
#include <std_msgs/Float64.h>

namespace vesc_ackermann {

    template<typename T>
    inline bool getRequiredParam(const ros::NodeHandle &nh, std::string name, T &value);

    AckermannToVesc::AckermannToVesc(ros::NodeHandle nh, ros::NodeHandle private_nh) {
        // get conversion parameters
        if (!getRequiredParam(nh, "speed_to_erpm_gain", speed_to_erpm_gain_))
            return;
        if (!getRequiredParam(nh, "speed_to_erpm_offset", speed_to_erpm_offset_))
            return;
        if (!getRequiredParam(nh, "duty_min", duty_min_))
            return;
        if (!getRequiredParam(nh, "duty_max", duty_max_))
            return;
        if (!getRequiredParam(nh, "duty_gain", duty_gain_))
            return;
        if (!getRequiredParam(nh, "steering_angle_to_servo_gain", steering_to_servo_gain_))
            return;
        if (!getRequiredParam(nh, "steering_angle_to_servo_offset", steering_to_servo_offset_))
            return;

        // create publishers to vesc electric-RPM (speed) and servo commands
        erpm_pub_ = nh.advertise<std_msgs::Float64>("commands/motor/speed", 10);
        duty_pub_ = nh.advertise<std_msgs::Float64>("commands/motor/duty_cycle", 10);
        servo_pub_ = nh.advertise<std_msgs::Float64>("commands/servo/position", 10);

        // subscribe to ackermann topic
        ackermann_sub_ = nh.subscribe("/vesc/low_level/ackermann_cmd_mux/output", 10, &AckermannToVesc::ackermannCmdCallback, this);
    }

    typedef ackermann_msgs::AckermannDriveStamped::ConstPtr AckermannMsgPtr;

    void AckermannToVesc::ackermannCmdCallback(const AckermannMsgPtr &cmd) {
        std_msgs::Float64::Ptr erpm_msg(new std_msgs::Float64);
        std_msgs::Float64::Ptr duty_msg(new std_msgs::Float64);
        std_msgs::Float64::Ptr servo_msg(new std_msgs::Float64);
        servo_msg->data = steering_to_servo_gain_ * cmd->drive.steering_angle + steering_to_servo_offset_;

        // publish
        if (ros::ok()) {
            if (cmd->drive.speed < -duty_min_  && cmd->drive.speed >= -duty_max_) {
                duty_msg->data = -duty_min_ + (cmd->drive.speed * duty_gain_);
                duty_pub_.publish(duty_msg);
            } else if (cmd->drive.speed > duty_min_ && cmd->drive.speed < duty_max_) {
                duty_msg->data = duty_min_ + (cmd->drive.speed * duty_gain_);
                duty_pub_.publish(duty_msg);
            } else if (cmd->drive.speed >= duty_max_) {
                erpm_msg->data = speed_to_erpm_gain_ * cmd->drive.speed + speed_to_erpm_offset_;
                erpm_pub_.publish(erpm_msg);
            } else {
                duty_msg->data = 0;
                duty_pub_.publish(duty_msg);
            }
            servo_pub_.publish(servo_msg);
        }
    }

    template<typename T>
    inline bool getRequiredParam(const ros::NodeHandle &nh, std::string name, T &value) {
        if (nh.getParam(name, value))
            return true;

        ROS_FATAL("AckermannToVesc: Parameter %s is required.", name.c_str());
        return false;
    }

} // namespace vesc_ackermann
