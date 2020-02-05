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

#include <chrono>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

float angular = 0.0;
float linear = 0.0;
float posX = 0.0, posY = 0.0, yaw = 0.0;


void rotate (double angular_speed, double desired_angle, ros::Publisher &vel_pub);

float minLaserDist[3] = {std::numeric_limits<float>::infinity(),std::numeric_limits<float>::infinity(),std::numeric_limits<float>::infinity()};
int32_t nLasers=0, desiredNLasers=0, desiredAngle=25, edgeRange= 100;

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	bumper[msg->bumper]=msg->state;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	minLaserDist[0] = std::numeric_limits<float>::infinity();
	minLaserDist[1] = std::numeric_limits<float>::infinity();
	minLaserDist[2] = std::numeric_limits<float>::infinity();
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = desiredAngle*M_PI / (180*msg->angle_increment);
    ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);
    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min) {
        uint32_t start_index = nLasers/2 - desiredNLasers;
        uint32_t increment_size = 2*desiredNLasers/3;
        for (uint32_t index_multiple = 0; index_multiple < 3; index_multiple++){
            uint32_t local_start_index = start_index + increment_size*(index_multiple);
            uint32_t local_end_index = start_index + increment_size*(index_multiple+1);
            for (uint32_t laser_idx = local_start_index; laser_idx < local_end_index; laser_idx++){
                minLaserDist[index_multiple] = std::min(minLaserDist[index_multiple], msg->ranges[laser_idx]);
            }
        }
        ROS_INFO("Left: %f, Center: %f Right: %f", minLaserDist[2],minLaserDist[1],minLaserDist[0]);
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

void checkBumperPressed(uint8_t bumper[3],ros::Publisher &vel_pub)
{
	if(bumper[1]==1){
        moveDist(-0.25,vel_pub);
	}
	else if(bumper[0]==1){
		moveDist(-0.25,vel_pub);
        rotate(1,30,1,vel_pub);
	}
	else if(bumper[2]==1){
        moveDist(-0.25,vel_pub);
        rotate(1,30,0,vel_pub);
	}
}

void moveDist(float targdist,ros::Publisher &vel_pub) 
{
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

void rotate(double angular_speed, double angle_desired, bool isLeft, ros::Publisher &vel_pub)
{
    geometry_msgs::Twist vel;
    double orig_angle, angle_rotated = 0.0;
    orig_angle = yaw; // left pi to right -pi
    ros::Rate loop_rate(10);
    while(abs(abs(angle_rotated) - abs(DEG2RAD(angle_desired))) > 0.01)
    {
        if (isLeft == 1){
            angle_rotated = yaw-orig_angle;
        }
        else if (isLeft == 0){
            angle_rotated = orig_angle-yaw;
        }
        
        vel.linear.x = 0.0;
        vel.linear.y = 0.0;
        vel.linear.z = 0.0;
        vel.angular.x = 0.0;
        vel.angular.y = 0.0;
        vel.angular = angular_speed;
        vel_pub.publish(vel);
        ros::spinOnce();
        loop_rate.sleep();
    }

}

void moveSpeed(double linear_speed, ros::Publisher &vel_pub)
{
   geometry_msgs::Twist vel;
   vel.angular.x = 0.0;
   vel.angular.y = 0.0;
   vel.angular.z = 0.0;
   vel.linear.y = 0.0;
   vel.linear.z = 0.0;
   vel.linear.x = linear_speed;
   vel_pub.publish(vel);
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
        checkBumperPressed(bumper,vel_pub);
        
        ROS_INFO("Postion: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, RAD2DEG(yaw), minLaserDist[1]);
		
        if (any_bumper_pressed) {
			bumperPressed();
		}
		else if (minLaserDist[0] > 0.5 && minLaserDist[1] > 0.5 && minLaserDist[2] > 0.5) {
			//determine laser sectors
		}
		else {
			linear = 0;
			angular = 0;
		}

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}
