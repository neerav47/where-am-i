#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

ros::Publisher motor_command_publisher;
//Define Call back method - handle_drive_request which takes our custom defined message type.
bool handle_drive_request(ball_chaser::DriveToTarget::Request& request, ball_chaser::DriveToTarget::Response& response){
    //Print Info messages
    ROS_INFO("DriveToTarget Request received - linear_x: %1.2f, angular_z: %1.2f", request.linear_x, request.angular_z);
    //Create geometry_msgs::Twist message
    geometry_msgs::Twist motor_command;
    //set linear and angular velocities.
    //geometry_msgs::Twist message type: linear: x y z angular x y z
    motor_command.linear.x = request.linear_x;
    motor_command.angular.z = request.angular_z;
    //Publish message using motor_command_publisher
    motor_command_publisher.publish(motor_command);
    //build response message string
    response.msg_feedback = "Targets to Drive linear_x: "+ std::to_string(request.linear_x)+", angular_z: "
                            + std::to_string(request.angular_z);
    return true;
}
int main(int argc, char** argv){
    //intialize
    ros::init(argc, argv, "drive_bot");
    //Create ROS NodeHandler object
    ros::NodeHandle n;
    //Inform ROS master that we will publishing message of Type Twist
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    //Define ball_chaser/command_robot service
    ros::ServiceServer ss = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
    //Handle ROS communication events
    ros::spin();
    return 0;

}

