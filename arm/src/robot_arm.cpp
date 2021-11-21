#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <vector>
#include <iostream>
 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "arm");
	ros::AsyncSpinner spinner(1);
	spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("arm");
  
    std::string end_effector_link = arm.getEndEffectorLink(); //Get the name of the terminal link
    std::cout<<"end_effector_link: "<<end_effector_link<<std::endl;
   
    std::string reference_frame = "/base_link"; //Set the reference coordinate system used by the target position
    arm.setPoseReferenceFrame(reference_frame);

    arm.allowReplanning(true); //When motion planning fails, re-planning is allowed
    arm.setGoalJointTolerance(0.001);
    arm.setGoalPositionTolerance(0.001); //Set the allowable error of position (unit: meter) and attitude (unit: radians)
    arm.setGoalOrientationTolerance(0.01);   
    arm.setMaxAccelerationScalingFactor(0.2); //Set the maximum speed and acceleration allowed
    arm.setMaxVelocityScalingFactor(0.2);

    geometry_msgs::Pose now_pose = arm.getCurrentPose(end_effector_link).pose;
    std::cout<<"now Robot position: [x,y,z]: ["<<now_pose.position.x<<","<<now_pose.position.y<<","<<now_pose.position.z<<"]"<<std::endl;
    std::cout<<"now Robot orientation: [x,y,z,w]: ["<<now_pose.orientation.x<<","<<now_pose.orientation.y<<","<<now_pose.orientation.z
       <<","<<now_pose.orientation.w<<"]"<<std::endl;
  
    arm.setNamedTarget("home");// Control the robotic arm to return to the initial position
    arm.move();
    sleep(1);

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose pose1;
    pose1.position.x = 0.1;
	pose1.position.y = 0.20212;	
	pose1.position.z = 2.16729;
	pose1.orientation.x = 0.70717;
	pose1.orientation.y = 0.707044;
	pose1.orientation.z = 5.42317e-05;
	pose1.orientation.w = 3.19771e-05;
	waypoints.push_back(pose1);

    geometry_msgs::Pose pose2;
    pose2.position.x = 0.251989;
	pose2.position.y = 0.578917;	
	pose2.position.z = 1.3212;
	pose2.orientation.x = 0.907055;
	pose2.orientation.y = 0.407159;
	pose2.orientation.z = 4.39101e-05;
	pose2.orientation.w = 8.60218e-05;
	waypoints.push_back(pose2);

	// Path planning in Cartesian space
	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.002;
	double fraction = 0.0;
    int maxtries = 100;   //Maximum number of planning attempts
    int attempts = 0;     //Number of planning attempts

    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        attempts++;
        
        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }
    
    if(fraction == 1)
    {   
        ROS_INFO("Path computed successfully. Moving the arm.");

	    // Generate motion planning data of the robotic arm
	    moveit::planning_interface::MoveGroupInterface::Plan plan;
	    plan.trajectory_ = trajectory;
	    // Perform movement
	    arm.execute(plan);
        
        sleep(1);
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }
    
	ros::shutdown(); 
	return 0;
}

