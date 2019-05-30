#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"
#include "sensor_msgs/Image.h"

//Ros::Publisher motor command
ros::ServiceClient client;


void drive_robot(float linear_x, float angular_z){
    //Initialize DriveTOtarget message type.
    ball_chaser::DriveToTarget target;
    target.request.linear_x = linear_x;
    target.request.angular_z = angular_z;
    client.call(target);
}
void process_image_callback(const sensor_msgs::Image img){
    int start = img.data.size() / 3;
    int end = img.data.size() * 2/3;
    int white_pixels = 0;
    int position = 0;
    float linear_x, angular_z;
    for(int i = start; i < end; i+=3){
        int red = img.data[i];
        int green = img.data[i+1];
        int blue = img.data[i+2];
        //Check if all three chanells are 255 - White
        if(red == 255 && green == 255 && blue == 255){
            white_pixels++;
            int initial = (i % (img.width * 3)) / 3;
            position += initial;
        }
    }

    //If number of white pixels is zero then no movement
    if(white_pixels != 0){
        int average = position / white_pixels;
        //average falls under left side
        if(average < img.width / 3){
            linear_x = 0.5;
            angular_z = 0.5;
        }else if(average > img.width * 2/3){ //average falls under right side
            linear_x = 0.5;
            angular_z = -0.5;
        }else{//Move forward
            linear_x = 0.5;
            angular_z = 0.0; 
        }
    }else{//If no wihte pixel found then stop.
        linear_x = 0.0;
        angular_z = 0.0;
    }
    //Command robot. Call ball_chaser/command_robot service.
    drive_robot(linear_x, angular_z);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    //Subscribe to camera topic
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    //Handle ROS communication events
    ros::spin();
    return 0;
}