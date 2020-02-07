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

//ros::Publisher vel_pub;
geometry_msgs::Twist vel;


float minLaserDist[12] = {};
float avgLaserDist[3] = {};
int32_t nLasers=0, desiredNLasers=0, desiredAngle=25, edgeRange= 100;

int32_t nLasers=0, desiredNLasers=0, desiredAngle=10, edgeRange= 100;

float left_d, right_d, center_d;
float reg_speed = 0.25;
float turn_speed = 0.1;
float ang_speed = 0.3;

int corner_counter = 0;
bool turned = false; 

//test obstacle distance
float wf_dis = 0.9;
float f_limit = 0.9; 
float l_limit = 1.5; 

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

    // nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    // desiredNLasers = desiredAngle*M_PI / (180*msg->angle_increment);
	//ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);
	
	nLasers = 639;
	desiredNLasers = 639;
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
            if (minLaserDist[j] == std::numeric_limits<float>::infinity()){ // infinity filtering new
                minLaserDist[j] = 0.0;
            }
            local_avg += minLaserDist[j];
        }

        avgLaserDist[i] = (float) local_avg / (float) 4;
    }
    ROS_INFO("AVG Left: %f, Center: %f Right: %f", avgLaserDist[2],avgLaserDist[1],avgLaserDist[0]);


}

void odomCallback(const nav_msgs::Odometry::ConstPtr&msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    //ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
}

void moveDist(float targdist,ros::Publisher &vel_pub) 
{
    geometry_msgs::Twist vel;
    ros::Rate loop_rate(10);
    float prev_posX, prev_posY, distMoved = 0.0;
	prev_posX = posX;
	prev_posY = posY;
	if (targdist>0){
		while(distMoved<targdist){
			//ROS_INFO("Moving : %f", distMoved);
			distMoved = sqrt(pow((posX - prev_posX),2)+pow((posY - prev_posY),2));
			vel.angular.z = 0;
			vel.linear.x = 0.2;
			vel_pub.publish(vel);
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	else if (targdist<0){
		while(distMoved<targdist){
			//ROS_INFO("Moving : %f", distMoved);
			distMoved = sqrt(pow((posX - prev_posX),2)+pow((posY - prev_posY),2));
			vel.angular.z = 0;
			vel.linear.x = -0.15;
			vel_pub.publish(vel);
			ros::spinOnce();
			loop_rate.sleep();
		}
	}

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

	if (angular_speed < 0){ // fix negative sign of while loop
		angular_speed = - angular_speed;
	}

    ros::Rate loop_rate(10);
    double current_angle = 0.0;
    double initial_time = ros::WallTime::now().toSec();
	std::cout << "inside the while" << std::endl;
    while(current_angle < DEG2RAD(desired_angle))
    {
       //std::cout<<current_angle<<std::endl;
       //ROS_INFO("current angle: %f", current_angle);
       vel_pub.publish(vel);
       double current_time = ros::WallTime::now().toSec();
       current_angle = angular_speed * (current_time - initial_time);
      //ros::spinOnce();
       loop_rate.sleep();
       
    }
	std::cout << "out the while" << std::endl;
    vel.angular.z = 0.0;
    vel_pub.publish(vel);
}

void moveSpeed(double linear_speed, ros::Publisher &vel_pub)
{
   geometry_msgs::Twist vel;
//    vel.angular.x = 0.0;
//    vel.angular.y = 0.0;
//    vel.angular.z = 0.0;
//    vel.linear.y = 0.0;
//    vel.linear.z = 0.0;
   vel.linear.x = linear_speed;
   vel_pub.publish(vel);
}

bool checkBumperPressed(uint8_t bumper[3],ros::Publisher &vel_pub)
{
	float backDist = -0.05, fwdDist = 0.01, rotAngle = 30.0;
	if(bumper[0]==1){ //check left
		ROS_INFO("Left bumper!\n");
        moveDist(backDist, vel_pub);
        rotate(-1.0,rotAngle, vel_pub);
		moveDist(fwdDist, vel_pub);
        return true;
	}
	else if(bumper[2]==1){ //check right
		ROS_INFO("Right bumper!\n");
        moveDist(backDist, vel_pub);
        rotate(1.0, rotAngle, vel_pub);
		moveDist(fwdDist, vel_pub);
        return true;
	}
	else if(bumper[1]==1){//check center
		ROS_INFO("Center bumper!\n");
        moveDist(-0.1, vel_pub);
		if(bumper[0]==1){ //check left
			rotate(-1.0,rotAngle, vel_pub);
			moveDist(fwdDist, vel_pub);
			return true;
		}
		else if(bumper[2]==1){ //check right
			rotate(1.0, rotAngle, vel_pub);
			moveDist(fwdDist, vel_pub);
			return true;
		}
        return true;
	}
    else{
		return false;
	}
}

typedef enum _ROBOT_MOVEMENT {
	STOP = 0,
	FORWARD,
	BACKWARD,
	TURN_LEFT,
	TURN_RIGHT,
	GO_RIGHT,
	GO_LEFT
} ROBOT_MOVEMENT;

bool robot_move(const ROBOT_MOVEMENT move_type,ros::Publisher &vel_pub) {
	
	if (move_type == STOP) {
		ROS_INFO("[ROBOT] STOP! \n");

		vel.linear.x = 0.0;
		vel.angular.z = 0.0;
	}

	else if (move_type == FORWARD) {
		ROS_INFO("[ROBOT] FORWARD! \n");

		vel.linear.x = 0.25;
		vel.angular.z = 0.0;
	}

	else if (move_type == BACKWARD) {
		ROS_INFO("[ROBOT] BACKWARD! \n");

		vel.linear.x = -0.25;
		vel.angular.z = 0.0 ;
	}

	else if (move_type == TURN_LEFT) {
		ROS_INFO("[ROBOT] TURN LEFT! \n"); 

		vel.linear.x = 0.0;
		vel.angular.z = 0.1;
	}

	else if (move_type == TURN_RIGHT) {
		ROS_INFO("[ROBOT] TURN RIGHT! \n");

		vel.linear.x = 0.0;
		vel.angular.z = -0.1;
	}

	else if (move_type == GO_LEFT) {
		ROS_INFO("[ROBOT] GO LEFT! \n");

		vel.linear.x = 0.25;
		vel.angular.z = 0.05;
	}

	else if (move_type == GO_RIGHT) {
		ROS_INFO("[ROBOT] GO RIGHT! \n");

		vel.linear.x = 0.25;
		vel.angular.z = -0.05;
	}
	else {
		ROS_INFO("[ROBOT_MOVE] WRONG! \n");
		return false;
	}

	vel_pub.publish(vel);
	usleep(10);
	return true;
}

int state_Check() {
	// 1 = find wall, 2 = 
	if (center_d < f_limit && left_d > l_limit&& right_d < l_limit) {
		return 1;
	}

	else if (center_d < f_limit && left_d < l_limit && right_d > l_limit) {
		return 1;
	}

	else if (center_d < f_limit && left_d > l_limit&& right_d > l_limit) {
		return 1;
	}

	else if (center_d < f_limit && left_d < l_limit && right_d < l_limit) {
		return 1;
	}

	else if (center_d > f_limit&& left_d < l_limit && right_d > l_limit) {
		return 2;
	}

	else if (center_d > f_limit&& left_d > l_limit&& right_d < l_limit) {
		return 2;
	}

	else if (center_d > f_limit&& left_d < l_limit && right_d < l_limit) {
		return 2;
	}

	//center_d > f_limit && left_d > l_limit && right_d > l_limit

	else {
		return 0;
	}
}

//wall_dir = 0 -> rightwall, wall_dir = 1 -> leftwall
int wallFollow(bool wall_dir,ros::Publisher &vel_pub) {
	geometry_msgs::Twist vel;
	float side;

	if (wall_dir) {//left
		
		side = left_d;
	}
	else {//right
		side = right_d;
	}

	if (center_d >= wf_dis && side < wf_dis) {
		robot_move(FORWARD,vel_pub);
		turned = false;
	}

	else if (center_d < wf_dis && side < wf_dis) {
		if (!turned) {
			corner_counter++;
			turned = true;
		}
		
		if (wall_dir) {
			robot_move(TURN_RIGHT,vel_pub);
		}
		else {
			robot_move(TURN_LEFT,vel_pub);
		}
	}
	else if (side >= wf_dis) {
		if (wall_dir) {
			robot_move(GO_LEFT,vel_pub);
		}
		else {
			robot_move(GO_RIGHT,vel_pub);
		}
	}
	return corner_counter;
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

    
    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();

        if (!checkBumperPressed(bumper,vel_pub)){
			
			int state = state_Check();

			//find a wall
			if (state == 0) {
				ROS_INFO("find a wall\n");
				robot_move(FORWARD,vel_pub);
			}

			//align
			else if (state == 1) {
				ROS_INFO("align \n");
				robot_move(TURN_LEFT,vel_pub);
			}

			//follow wall
			else {
				ROS_INFO("follow wall\n");
				std::chrono::time_point<std::chrono::system_clock> start_cycle;
				start_cycle = std::chrono::system_clock::now();
				uint64_t cycle_time_limit = random() % 10 + 25;
				uint64_t cycle_time = 0;

				int turn_dir;

				turn_dir = rand() % 2;
					
				if (turn_dir) {
					ROS_INFO("rotate 180\n");
					rotate(0.2, 180.0, vel_pub);
					ros::spinOnce();
				}
				
				corner_counter = wallFollow(turn_dir,vel_pub);
				
				// while (cycle_time <= cycle_time_limit || corner_counter < 3) {
				// 	cycle_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start_cycle).count();


				// 	corner_counter = wallFollow(turn_dir,vel_pub);
				// 	ros::spinOnce();
				// 	checkBumperPressed(bumper,vel_pub);
				// 	if (cycle_time > 15 && corner_counter < 1)
				// 		break;
				// }

				double deg = rand() % 61 + 60;
				if (turn_dir) {
					ROS_INFO("random turn right\n");
					rotate(0.2, -deg, vel_pub);
				}
				else {
					ROS_INFO("random turn left\n");
					rotate(0.2, deg, vel_pub);
				}
			}
        }

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}
