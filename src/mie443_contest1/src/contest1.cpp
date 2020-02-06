#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)

#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>

#include <stdio.h>
#include <cmath>
#include <math.h>

#include <chrono>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

float angular = 0.0;
float linear = 0.0;
float posX = 0.0, posY = 0.0, yaw = 0.0;

void rotate (double angular_speed, double desired_angle, ros::Publisher &vel_pub);

float minLaserDist[12] = {};
float avgLaserDist[3] = {};
int32_t nLasers=0, desiredNLasers=0, desiredAngle=25, edgeRange= 100;

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	bumper[msg->bumper]=msg->state;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // define laser and fill with float
	// minLaserDist[12] = {};
    std::fill(minLaserDist, minLaserDist+nLasers, std::numeric_limits<float>::infinity());

    // define average array
    // avgLaserDist[3] = {};
    std::fill(avgLaserDist, avgLaserDist+nLasers, std::numeric_limits<float>::infinity());

    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = desiredAngle*M_PI / (180*msg->angle_increment);
    ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);

    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min) {
        uint32_t start_index = nLasers/2 - desiredNLasers;
        uint32_t increment_size = 2*desiredNLasers/12;

        for (uint32_t index_multiple = 0; index_multiple < 12; index_multiple++){
            uint32_t local_start_index = start_index + increment_size*(index_multiple);
            uint32_t local_end_index = start_index + increment_size*(index_multiple+1);
            for (uint32_t laser_idx = local_start_index; laser_idx < local_end_index; laser_idx++){
                minLaserDist[index_multiple] = std::min(minLaserDist[index_multiple], msg->ranges[laser_idx]);
            }
        }

        int idx_start = 0;
        // now take the average per each 3 segment
        for (uint32_t i = 0; i < 3; i++){
            idx_start = idx_start + i*3;
            uint32_t idx_end = idx_start + (i+1)*3;
            float local_avg = 0.0;

            for (uint32_t j = idx_start; idx_end; j++){
                // infinity filtering
                // if (isinf(minLaserDist[j])){
                //     minLaserDist[j] = 0.0;
                // } 
                local_avg += minLaserDist[j];
            }

            avgLaserDist[i] = (float) local_avg / (float) 4;
        }

        ROS_INFO("AVG Left: %f, Center: %f Right: %f", avgLaserDist[2],avgLaserDist[1],avgLaserDist[0]);
    }
    else {
        for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx) {
            minLaserDist[1] = std::min(minLaserDist[1], msg->ranges[laser_idx]);
        }
        ROS_INFO("(%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr&msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
}

void bumperPressed(){
	uint32_t b_idx; // why do I have to write it before
    for (b_idx = 0; b_idx < N_BUMPER; ++b_idx){
		if(b_idx == 0){
            //action
		}
		else if(b_idx == 1){
			//action
		}
		else if(b_idx == 2){
            //action
		}
	}
}

void moveDist(float targdist,ros::Publisher &vel_pub) {
    geometry_msgs::Twist vel;
    ros::Rate loop_rate(10);
    float prev_posX, prev_posY, distMoved = 0.0;
	prev_posX = posX;
	prev_posY = posY;
	while(distMoved<targdist){
		ROS_INFO("Moving : %f", distMoved);
        distMoved = sqrt(pow((posX - prev_posX),2)+pow((posY - prev_posY),2));
		vel.angular.z = 0;
		vel.linear.x = 0.25;
		vel_pub.publish(vel);
		ros::spinOnce();
        loop_rate.sleep();
	}
}

void moveTime(){
	//write moveTime fuction here
}

void rotate (double angular_speed, double desired_angle, ros::Publisher &vel_pub)
{
    geometry_msgs::Twist vel;
    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = angular_speed;
//    if(clockwise==1)
//    {
//        vel.angular.z = -abs(DEG2RAD(angular_speed));
//    }
//    else
//    {
//        vel.angular.z = abs(DEG2RAD(angular_speed));
//    }
    ros::Rate loop_rate(10);
    double current_angle = 0.0;
    double initial_time = ros::WallTime::now().toSec();
    while(current_angle < DEG2RAD(desired_angle))
        {
        //std::cout<<current_angle<<std::endl;
        ROS_INFO("current angle: %f", current_angle);
        vel_pub.publish(vel);
        double current_time = ros::WallTime::now().toSec();
        current_angle = angular_speed * (current_time - initial_time);
        ros::spinOnce();
        loop_rate.sleep();
        std::cout << "inside the while" << std::endl;
        }
        vel.angular.z = 0.0;
        vel_pub.publish(vel);
}

void move (double linear_speed, ros::Publisher &vel_pub)
{
   geometry_msgs::Twist vel;
   //ros::Rate loop_rate(10);
   vel.angular.x = 0.0;
   vel.angular.y = 0.0;
   vel.angular.z = 0.0;
   vel.linear.y = 0.0;
   vel.linear.z = 0.0;
   vel.linear.x = linear_speed;
//    if(forward)
//    {
//        vel.linear.x = -abs(linear_speed);
//    }
//    else
//    {
//        vel.linear.x = abs(linear_speed);
//    }
   vel_pub.publish(vel);
   //ros::spinOnce();
   //loop_rate.sleep();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
	ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    float angular = 0.0;
    float linear = 0.0;
    int asdf = 1;
    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        //fill with your code

        // laserCallback();

        // if (asdf==1){
        //     rotate(1, 120, vel_pub);
        //     moveDist(1,vel_pub);
        //     asdf++;
        // }
        
        bool any_bumper_pressed = false;
        for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
            any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
        }

        //Control logic after bumpers are being pressed.
        //ROS_INFO("Postion: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, RAD2DEG(yaw), minLaserDist[1]);
		// if (any_bumper_pressed) {
		// 	bumperPressed();
		// }
		// else if (minLaserDist[0] > 0.5 && minLaserDist[1] > 0.5 && minLaserDist[2] > 0.5) {
		// 	//determine laser sectors
		// }
		// else {
		// 	linear = 0;
		// 	angular = 0;
		// }

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}
