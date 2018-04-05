/*
 > @Global to Local WMR (Wheeled Mobile Robot) co-ordinate trsnsformation node
 > @file GPS_to_Pose.cpp
*/

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <string>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <wmr_state_feedback/GPS2xy.h>
#include <wmr_pkg/SubscribeTopic.h>

using namespace wmr_state_feedback;

//main function

int main(int argc, char **argv)
{
	GPS2xy gps_to_xy;
	
		ros::init(argc, argv, "gps_to_xy"); // initialising node with name "gps2xy"
		
		ros::NodeHandle n; // creating nodehandle

	std::string gps_pub_topic;
	if(!n.getParam("wmr_state_feedback/gps/pose_topic", gps_pub_topic))  
	{ gps_pub_topic = "wmr_state_feedback/position_from_gps"; }

	// Subscriber Objects

		ros::Subscriber gps_sub = n.subscribe("mavros/global_position/global", 1000, 
						&GPS2xy::Gps_callback, &gps_to_xy); // Subscribing to the gps topic

		ros::Subscriber init_cond_set_sub = n.subscribe(gps_pub_topic+"/set_init_cond",10,
						&GPS2xy::init_cond_callback, &gps_to_xy); // Subscribing to the initial condition

	// Pubslish Objects
		
		ros::Publisher local_pose_pub = n.advertise< geometry_msgs::PointStamped > (gps_pub_topic,100); // publishing topic 
		
		wmr_pkg::SubscribeTopic <std_msgs::Bool> sim(n, "start_sim"); // WMR package

		
		ros::Rate loop_rate(5); // GPS data colloction at  5 HZ
		int count = 0;

		geometry_msgs::PointStamped local_pose; // Global variable initialization
		bool initiated = false;

		ros::spinOnce();


		ROS_INFO("\033[1;32mInitialized\033[0;m:= %s",ros::this_node::getName().c_str());
		
		// 
		while(ros::ok())
		{
		if ((sim.get_data())->data && !initiated) 
		{
			ROS_INFO("\033[1;32mStarted\033[0;m:= Local position from raw_GPS");
			gps_to_xy.reset();
			count = 0;
			initiated = true;
		}
		if (initiated && !(sim.get_data())->data)
		{
			ROS_INFO("\033[1;31mStopped\033[0;m:= Local Position from raw_GPS");
			initiated = false;
		}
		
		//Convert current Global to ECEF co-oprdiante
		gps_to_xy.Geod2ECEF();

		//Transform the ECEF frame to local ENU frame
		gps_to_xy.ECEF2ENU();
			

			// Publish Local Position
			local_pose.header.seq = count; //current sequence
			local_pose.header.stamp = ros::Time::now(); //current time of publish
			local_pose.header.frame_id = "local_ENU"; //frame in which (X,Y) is specified

			local_pose.point.x = gps_to_xy.Pose_ENU_x(); 
			local_pose.point.y = gps_to_xy.Pose_ENU_y();
			local_pose.point.z = gps_to_xy.Pose_ENU_z();
	
			local_pose_pub.publish(local_pose);

			ros::spinOnce(); // Referesh all Callbacks
			loop_rate.sleep();
			
		count++;
		}
}
