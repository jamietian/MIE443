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

#include <string>

class bumperNode
{ 
    public:
        // by default, activated is false
        bool activated = false;
        std::string activated_side;
}; 

class odomNode
{ 
    public:
        // by default, activated is false
        float X = 0.0;
        float Y = 0.0;
}; 

//odomotry parameters
float posX = 0.0, posY = 0.0, yaw = 0.0;
float distance;
int wallcount = 0;

// testing
int findWall_count = 0;

//bumper parameters
uint8_t bumper[3] = { kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED };

//laser parameters
float minLaserDist[11] = {};
float avgLaserDist[3] = {}; 
int32_t nLasers = 0, desiredNLasers = 0, desiredAngle = 10, edgeRange = 100;
float left_d, right_d, center_d;
float min_left, min_center, min_right;

//speed parameters
geometry_msgs::Twist vel;
float angular = 0.0;
float linear = 0.0;
float reg_speed = 0.16; //>1
float app_speed1 = 0.15 * 0.7; //0.7<x<=1
float app_speed2 = 0.15 * 0.5; // 0.55<x<=0.7
float app_speed3 = 0.15 * 0.3; //x<0.55 && x!=0
float ang_speed = 0.5; //turn and go
float turn_speed = 0.5; //turn

//obstacle distance
float f_safeDist = 1.0;
float f_limit = 0.7;
float emergency = 0.55;
float side_limit = 0.9;
float side_limint_low = 0.6;

//wall following parameters
int corner_counter = 0;
bool turned = false;
float side_dist_old = 1000; //initial large number

bool wallFound = false, aligned = false, dir_decided = false, newOB = false, finishLoop = true, openspace = true;

float initial_X, initial_Y,current_X, current_Y;

int turn_dir = 0;

// header ------- add all core function's header here
void moveDist(float targdist,ros::Publisher &vel_pub);
void rotate (double angular_speed, double desired_angle, ros::Publisher &vel_pub);
void repeatHandler(odomNode repeat_odomNode, ros::Publisher &vel_pub);

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	bumper[msg->bumper]=msg->state;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // define laser and fill with float
	// minLaserDist[12] = {};
    std::fill(std::begin(minLaserDist), std::end(minLaserDist), std::numeric_limits<float>::infinity());

    // define average array
    // avgLaserDist[3] = {};
    std::fill(std::begin(avgLaserDist), std::end(avgLaserDist), std::numeric_limits<float>::infinity());

    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = desiredAngle*M_PI / (180*msg->angle_increment);
	//ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);
	
	nLasers = 639;
	desiredNLasers = 639;
	// uint32_t start_index = nLasers/2 - desiredNLasers;
    uint32_t increment_size = desiredNLasers/11;

    for (uint32_t index_multiple = 0; index_multiple < 11; index_multiple++){
        uint32_t local_start_index = increment_size*(index_multiple);
        uint32_t local_end_index = increment_size*(index_multiple+1);
        for (uint32_t laser_idx = local_start_index; laser_idx < local_end_index; laser_idx++){
            minLaserDist[index_multiple] = std::min(minLaserDist[index_multiple], msg->ranges[laser_idx]);
        }
    }
	//ROS_INFO("Left: %f, Center: %f, Right: %f", minLaserDist[10],minLaserDist[5],minLaserDist[0]);
    min_left = minLaserDist[10];
	min_center = minLaserDist[5];
	min_right = minLaserDist[0];
	
	uint16_t idx_start = 0;
	uint16_t idx_end = 0;
    //now take the average per each 3 segment
   	for (uint16_t i = 0; i < 3; i++){
		if (i==0){
			idx_start = 0;
			idx_end = 2;
		}
		else if (i == 1){
			idx_start = 4;
			idx_end = 6;
		}
		else if (i == 2){
			idx_start = 8;
			idx_end = 10;
		}
		float local_avg = 0.0;
        for (uint32_t j = idx_start; j <= idx_end; j++){
            if (minLaserDist[j] == std::numeric_limits<float>::infinity()){ // infinity filtering new
                minLaserDist[j] = 0.0;
            }
            local_avg += minLaserDist[j];
        }

        avgLaserDist[i] = (float) local_avg / (float) 3;
    }
    //ROS_INFO("AVG Left: %f, Center: %f Right: %f", avgLaserDist[2],avgLaserDist[1],avgLaserDist[0]);
	left_d = avgLaserDist[2];
    center_d = avgLaserDist[1];
	right_d = avgLaserDist[0];
}


void odomCallback(const nav_msgs::Odometry::ConstPtr&msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    //ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
}



//type of movement
typedef enum _ROBOT_MOVEMENT {
	STOP = 0,
	FORWARD,
	BACKWARD,
	TURN_LEFT,
	TURN_RIGHT,
	GO_RIGHT,
	GO_LEFT
} ROBOT_MOVEMENT;

void robot_move(const ROBOT_MOVEMENT move_type, float speed, ros::Publisher &vel_pub, float go_turn = ang_speed) {
	
	if (move_type == STOP) {
		//ROS_INFO("[ROBOT] STOP! \n");
		vel.linear.x = 0.0;
		vel.angular.z = 0.0;
	}

	else if (move_type == FORWARD) {
		//ROS_INFO("[ROBOT] FORWARD! \n");
		vel.linear.x = speed;
		vel.angular.z = 0.0;
	}

	else if (move_type == BACKWARD) {
		//ROS_INFO("[ROBOT] BACKWARD! \n");
		vel.linear.x = -0.8*speed;
		vel.angular.z = 0.0;
	}

	else if (move_type == TURN_LEFT) {
		//ROS_INFO("[ROBOT] TURN LEFT! \n");
		vel.linear.x = 0.0;
		vel.angular.z = go_turn;
	}

	else if (move_type == TURN_RIGHT) {
		//ROS_INFO("[ROBOT] TURN RIGHT! \n");
		vel.linear.x = 0.0;
		vel.angular.z = -go_turn;
	}

	else if (move_type == GO_LEFT) {
		//ROS_INFO("[ROBOT] GO LEFT! \n");
		vel.linear.x = speed;
		vel.angular.z = go_turn;
	}

	else if (move_type == GO_RIGHT) {
		//ROS_INFO("[ROBOT] GO RIGHT! \n");
		vel.linear.x = speed;
		vel.angular.z = -go_turn;
	}
	else {
		ROS_INFO("[ROBOT_MOVE] WRONG! \n");
	}

	vel_pub.publish(vel);
	usleep(10);
}

bool checkLoop(){
	if (wallcount > 2) {
		current_X = posX;
		current_Y = posY;
		distance = sqrt(pow((current_X - initial_X), 2) + pow((current_Y - initial_Y), 2));
		ROS_INFO("Distance: %f", distance);
		// originally 0.2
		if (distance < 0.3) {
			ROS_INFO("finish loop");
			openspace = true;
			finishLoop = true;
			return true;
		}
		else {
			return false;
		}
	}
	else {
		return false;
	}
}

void moveDist(float targdist,ros::Publisher &vel_pub) 
{
    ros::Rate loop_rate(10);
    float prev_posX, prev_posY, distMoved = 0.0;
	prev_posX = posX;
	prev_posY = posY;
	if (targdist > 0) {
		while (distMoved < targdist) {
			//ROS_INFO("Moving : %f", distMoved);			
			distMoved = sqrt(pow((posX - prev_posX), 2) + pow((posY - prev_posY), 2));
			vel.angular.z = 0;
			vel.linear.x = reg_speed;
			vel_pub.publish(vel);
			ros::spinOnce();
			loop_rate.sleep();

			if (bumper[0] == 1 || bumper[1] == 1 || bumper[2] == 1) {
				robot_move(BACKWARD, app_speed1, vel_pub);
				return;
			}		
		}
	}
	else if (targdist < 0) {
        targdist =-targdist;
		while (distMoved < targdist) {
			//ROS_INFO("Moving : %f", distMoved);
			/*if (bumper[0] == 1 || bumper[1]==1 || bumper[2]==1){
				robot_move(BACKWARD,app_speed1, vel_pub);
				return;
			}*/
			distMoved = sqrt(pow((posX - prev_posX), 2) + pow((posY - prev_posY), 2));
			vel.angular.z = 0;
			vel.linear.x = -app_speed1;
			vel_pub.publish(vel);
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
}

void rotate (double angular_speed, double desired_angle, ros::Publisher &vel_pub)
{
    vel.linear.x = 0.0;
    vel.angular.z = -angular_speed;

	if (angular_speed < 0) { // fix negative sign of while loop
		angular_speed = -angular_speed;
	}

    ros::Rate loop_rate(10);
    double current_angle = 0.0;
    double initial_time = ros::WallTime::now().toSec();
	//std::cout << "inside the rotete while loop" << std::endl;
    while(current_angle < DEG2RAD(desired_angle))
    {
       //std::cout<<current_angle<<std::endl;
       //ROS_INFO("current angle: %f", current_angle);
       vel_pub.publish(vel);
       double current_time = ros::WallTime::now().toSec();
       current_angle = angular_speed * (current_time - initial_time);
       loop_rate.sleep(); 
    }
	//std::cout << "out the rotate while loop" << std::endl;
    vel.angular.z = 0.0;
    vel_pub.publish(vel);
}


bool checkBumperPressed(uint8_t bumper[3],ros::Publisher &vel_pub)
{
	float backDist = -0.1, fwdDist = 0.15, rotAngle = 30.0, rotSpeed = ang_speed;
	if(bumper[0]==1){ //check left
		ROS_INFO("Left bumper!\n");
        moveDist(backDist, vel_pub);
        rotate(rotSpeed,rotAngle, vel_pub);
		moveDist(fwdDist, vel_pub);
        return true;
	}
	else if(bumper[2]==1){ //check right
		ROS_INFO("Right bumper!\n");
        moveDist(backDist, vel_pub);
        rotate(-rotSpeed, rotAngle, vel_pub);
		moveDist(fwdDist, vel_pub);
        return true;
	}
	else if(bumper[1]==1){//check center
		ROS_INFO("Center bumper!\n");
        moveDist(backDist, vel_pub);
		if(bumper[0]==1){ //check left
			rotate(rotSpeed,rotAngle, vel_pub);
			moveDist(fwdDist, vel_pub);
			return true;
		}
		else if(bumper[2]==1){ //check right
			rotate(-rotSpeed, rotAngle, vel_pub);
			moveDist(fwdDist, vel_pub);
			return true;
		}
        return true;
	}
    else{
		return false;
	}
}


int state_Check() {
	/*
	state 0: empty space <- center, left, right all greater than limit
	state 1: wall found <- obstacle in front 
	state 2: follow wall
	*/

	if (finishLoop) {
		finishLoop = false; 
		ROS_INFO ("new loop");
		return 3;
	}
	
	else {
		// if (center_d < f_limit){ 
		if (center_d < f_limit){ 
			return 1;
		}

		else if (center_d > f_limit && (side_limint_low<left_d < side_limit || side_limint_low<right_d < side_limit)) {
			return 2;
		}


		else { //center_d > f_limit && left_d > l_limit && right_d > l_limit
			if (wallFound) {
				if (right_d > 1.5 * side_limit) {
					//ROS_INFO("threshold");
					return 0;
				}
				return 1;
			}
			else{
				return 0;

			}
			
		}
	}
}


float forward_speed_check(float center_d, float right_d, float left_d) {
	float linSpeed;
	if (left_d < emergency || right_d < emergency) {
		linSpeed = 0.0;
	}
	else {
		if (f_limit < center_d <= f_safeDist) //0.7<x<=1
			linSpeed = app_speed1;
		else if (emergency < center_d <= f_limit) //0.55 < x <= 0.7
			linSpeed = app_speed2;
		else if (emergency < center_d && center_d != 0) //x < 0.55 && x != 0
			linSpeed = app_speed3;
		else if (center_d == 0)
			linSpeed = 0.0;
		else
			linSpeed = reg_speed;
	}
	return linSpeed;
}


//wall_dir = 0 -> rightwall, wall_dir = 1 -> leftwall
void wallFollow(bool wall_dir,ros::Publisher &vel_pub) {

	// only when new wall is followed, you store initial_X and initial_Y for odom
	if (newOB) {
		initial_X = posX;
		initial_Y = posY;
		newOB = false;
		ROS_INFO("start position: x: %f, y: %f \n", initial_X, initial_Y);
        openspace = false;
	}

	float side;

	if (wall_dir) //left	
		side = left_d;
	else //right
		side = right_d;

	if (side - side_dist_old > 0.08) {
        ROS_INFO("moving away! adjust");
		if(wall_dir) 
			robot_move(GO_LEFT, reg_speed, vel_pub, 0.5*ang_speed);
		else
			robot_move(GO_RIGHT, reg_speed, vel_pub, 0.5 * ang_speed);
	}
	else if (side_dist_old - side > 0.08) {
        ROS_INFO("getting closer! adjust");

		if (wall_dir)
			robot_move(GO_RIGHT, reg_speed, vel_pub, 0.5 * ang_speed);
		else
			robot_move(GO_LEFT, reg_speed, vel_pub, 0.5 * ang_speed);
	}
	else
	{
		robot_move(FORWARD, reg_speed, vel_pub);
	}
     side_dist_old = side;
}


void action(int state,ros::Publisher &vel_pub) {
	float speed;

	//find a wall
	if (state == 0) {
		if (wallFound) { //going to find a new wall
			//dir_decided = false;
			wallFound = false;
			wallcount++;
		}
		//if (!dir_decided) {
		//	turn_dir = rand() % 2;
		//  dir_decided = true;
		//}
		ROS_INFO("go & find wall\n");
		//speed = forward_speed_check(center_d, right_d, left_d);
        //moveDist(0.1,vel_pub) ;
		//if (turn_dir) 
		//	robot_move(GO_LEFT, reg_speed, vel_pub);
		//else
			//ROS_INFO("go & turn\n");
        if (openspace){
            ROS_INFO("openspace true");
            robot_move(FORWARD, reg_speed, vel_pub);
        }
        else {
            if (findWall_count < 8) {
                ROS_INFO("yeet");
                robot_move(FORWARD, reg_speed, vel_pub);
                findWall_count += 1;
            }
            else{
                ROS_INFO("openspace false");
                robot_move(GO_RIGHT, reg_speed*0.8, vel_pub, 0.5*ang_speed);
            }
            
        }
			
	}

	//align
	else if (state == 1) {
        findWall_count = 0;
		wallFound = true;
        openspace = false;
		ROS_INFO("align \n");
		//if (turn_dir) 
		//	robot_move(TURN_RIGHT, 0.0, vel_pub);
		//else 
		robot_move(TURN_LEFT, 0.0, vel_pub, 0.4);	
	}
	
	//follow wall
	else if (state == 2){
        findWall_count = 0;
		wallFound = true;
		ROS_INFO("follow \n");

		wallFollow(turn_dir, vel_pub);
	}
	
	else {
		newOB = true; 
		wallcount = 0; 
		double deg = rand() % 60 + 60;
		ROS_INFO("rotate %f to find a new obstacle\n", deg);
		rotate(-turn_speed, deg, vel_pub);
		moveDist(0.5, vel_pub);
	}
}

void repeatHandler(odomNode repeat_odomNode, ros::Publisher &vel_pub) {
	float repeatDistDiff = sqrt(pow((posX - repeat_odomNode.X), 2) + pow((posY - repeat_odomNode.Y), 2));

	// if for the past 30 seconds, you have not moved away by 30 cms
	if (repeatDistDiff < 0.3) {

		ROS_INFO("repeating movements");
		float openDist = std::max({left_d, center_d, right_d});

		// if all lasers are low values, rotate 180 and move the other direction
		if (openDist < 0.5) {
			rotate(-turn_speed, 180, vel_pub);
			moveDist(0.3, vel_pub);
		}
		else if (openDist == left_d) {
			// rotate left and move forward - -> CCW
			rotate(-turn_speed, 30, vel_pub);
			moveDist(0.3, vel_pub);
		}
		else if (openDist == center_d) {
			// just move forward
			moveDist(0.3, vel_pub);
		}
		else /* openDist == right_d */ {
			// rotate right and move forward
			rotate(turn_speed, 30, vel_pub);
			moveDist(0.3, vel_pub);
		}

		// set open space true, to allow the robot to explore
		openspace = true;
	}
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

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    uint16_t cycle = 90;
	uint16_t repeat_cycle = 30;
    ros::WallTime cycle_start = ros::WallTime::now();
	ros::WallTime repeat_clock = ros::WallTime::now();

	int current_state;
	float current_speed;

	// initial odom distance store
    odomNode repeat_odomNode;
    repeat_odomNode.X = posX;
    repeat_odomNode.Y = posY;

    //main loop
    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();

       	// ROS_INFO("bumper checking outside: %i %i %i",bumper[0],bumper[1],bumper[2])
		
		if ((ros::WallTime::now() - repeat_clock).toSec() > repeat_cycle) {

			repeatHandler(repeat_odomNode,vel_pub);

			// refresh your clock and odomNode coordinates
			repeat_clock = ros::WallTime::now();
			repeat_odomNode.X = posX;
    		repeat_odomNode.Y = posY;
		}

        if (!checkBumperPressed(bumper,vel_pub)){
        	// ROS_INFO("bumper checking inside: %i %i %i",bumper[0],bumper[1],bumper[2]);
			if ((ros::WallTime::now() - cycle_start).toSec() > cycle) {
				ROS_INFO("360 check");
				rotate(-0.3,360,vel_pub);
				cycle_start = ros::WallTime::now();
			}

			// wall distance handler
			if (avgLaserDist[0] < 0.50) {
				// 2 degrees CCW
				rotate(-turn_speed*1.5, 2, vel_pub);
			}

			// first it checks loop to see if you are within the loop admissible stage
			// openSpace will be triggerred if foundLoop is True
			if (!openspace){
				bool foundLoop = checkLoop();
			}
			
			current_state = state_Check();
			action(current_state,vel_pub);
            
        }

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}