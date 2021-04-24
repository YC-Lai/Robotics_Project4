#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "ball_chaser/DriveToTarget.h"

class process_image {
   private:
    ros::NodeHandle _n;
    ros::Subscriber _sub;
    ros::ServiceClient _client;

   public:
    process_image();
    void drive_robot(float lin_x, float ang_z);
    void process_image_callback(const sensor_msgs::Image img);
};

process_image::process_image() {
    _client = _n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
    _sub = _n.subscribe("/camera/rgb/image_raw", 10, &process_image::process_image_callback, this);
}

void process_image::drive_robot(float lin_x, float ang_z) {
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!_client.call(srv))
        ROS_ERROR("Failed to call service DriveToTarget");
}

void process_image::process_image_callback(const sensor_msgs::Image img) {
    int white_pixel = 255;

    float ball_in_l{0.0};
    float ball_in_r{0.0};
    float ball_in_c{0.0};

    uint32_t one_third = img.width / 3;
    uint32_t two_third = 2 * img.width / 3;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    img.data;
    for (size_t row = 0; row < img.height; row++) {
        for (size_t col = 0; col < img.width; col++) {
            auto pixel_r = img.data[row * img.width * 3 + col * 3 + 0];
            auto pixel_g = img.data[row * img.width * 3 + col * 3 + 1];
            auto pixel_b = img.data[row * img.width * 3 + col * 3 + 2];

            if (pixel_r == white_pixel &&
                pixel_g == white_pixel &&
                pixel_b == white_pixel) {
                if (col < one_third) {
                    ++ball_in_l;
                } else if (col > two_third) {
                    ++ball_in_r;
                } else {
                    ++ball_in_c;
                }
            }
        }
    }

    float ball_area = ball_in_l + ball_in_c + ball_in_r;
    float image_area = img.height * img.width;
    auto ball = ball_area / image_area;
    ROS_INFO("ball %1.5f", ball);
    if (ball < 0.15 && ball > 0) {
        if (ball_in_l / ball_area > 0.5) {
            ROS_INFO_STREAM("turn left...");
            drive_robot(0.5, 0.5);
        } else if (ball_in_r / ball_area > 0.5) {
            ROS_INFO_STREAM("turn right...");
            drive_robot(0.5, -0.5);
        } else {
            ROS_INFO_STREAM("drive forward...");
            drive_robot(0.5, 0);
        }

    } else {
        drive_robot(0.0, 0.0);
    }
}

int main(int argc, char** argv) {
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");

    process_image Object;

    // Handle ROS communication events
    ros::spin();

    return 0;
}