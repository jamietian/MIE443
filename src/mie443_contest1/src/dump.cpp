bumperNode checkBumperPressed(uint8_t bumper[3],ros::Publisher &vel_pub)
{
    /* action logic
    left bumper: move back --> rotate CW (right)
    right bumper: move back --> rotate CCW (left)
    center bumper: move back ONLY
    */ 

    // create bumper node, which will store bumper state
    bumperNode bnode;

    // static variable used in bumper checking process
	float backDist = -0.1, fwdDist = 0.15, rotAngle = 30.0, rotSpeed = ang_speed;

	if(bumper[0]==1){ //check left
		ROS_INFO("Left bumper!\n");
        bnode.activated_side = "left";
        bnode.activated = true;
        moveDist(backDist, vel_pub);
        rotate(rotSpeed,rotAngle, vel_pub);
		moveDist(fwdDist, vel_pub);
	}
	else if(bumper[2]==1){ //check right
		ROS_INFO("Right bumper!\n");
        bnode.activated_side = "right";
        bnode.activated = true;
        moveDist(backDist, vel_pub);
        rotate(-rotSpeed, rotAngle, vel_pub);
		moveDist(fwdDist, vel_pub);
	}
	else if(bumper[1]==1){//check center
        bnode.activated_side = "center";
        bnode.activated = true;
		ROS_INFO("Center bumper!\n");
        moveDist(backDist, vel_pub);
	}

    // bumper is never activated
    else{
		bnode.activated = false;
	}

    return bnode
}