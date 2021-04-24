#include <ball_chaser/DriveToTarget.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
//TODO: Include the ball_chaser "DriveToTarget" header file

// ROS::Publisher motor commands;
class drive_bot {
   public:
    drive_bot() {
        _motor_command_publisher = _n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        _service = _n.advertiseService("/ball_chaser/command_robot", &drive_bot::handle_drive_request, this);
    }

    bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
                              ball_chaser::DriveToTarget::Response& res) {
        ROS_INFO("motor_command received - linear_x: j1:%1.2f, angular_z: j2:%1.2f", (float)req.linear_x, (float)req.angular_z);
        // Create a motor_command object of type geometry_msgs::Twist
        geometry_msgs::Twist motor_command;
        // Set wheel velocities, forward [0.5, 0.0]
        motor_command.linear.x = req.linear_x;
        motor_command.angular.z = req.angular_z;

        _motor_command_publisher.publish(motor_command);
        return true;
    }

   private:
    ros::NodeHandle _n;
    ros::Publisher _motor_command_publisher;
    ros::ServiceServer _service;
};

int main(int argc, char** argv) {
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    drive_bot robot;

    ROS_INFO("Ready to send the commands");

    // TODO: Handle ROS communication events
    ros::spin();

    return 0;
}