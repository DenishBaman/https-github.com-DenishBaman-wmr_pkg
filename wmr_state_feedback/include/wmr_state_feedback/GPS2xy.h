
/*
 > @Script Header file
 > @brife  Global to Local co-ordinate conversion 
 > @file GPS2xy.h
 > C++ 14
*/
#ifndef _wmr_state_feedback_GPS2xy_H_
#define _wmr_state_feedback_GPS2xy_H_

#include <iostream>
#include <cmath>
#include <array>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/NavSatFix.h>

namespace wmr_state_feedback {

class GPS2xy{

	double lat_init = 0.0, lon_init = 0.0, alt_init = 0.0;
	double lat = 0.0, lon = 0.0, alt = 0.0;

	std::array<double, 3> Pose_ECEF_init{{0.0,0.0,0.0}},
			      Pose_ECEF{{0.0,0.0,0.0}},
			      Pose_ENU{{0.0,0.0,0.0}};
	
	const double earth_rad = 6371000; // radius of Earth (in m)
	const double semi_minor = 6356752.3142; // semi minor axis (in m)
	const double semi_major = 6378137; // semi major axis (in m)
	const double Eccen_sq = 0.0066943800; // Eccentricity squared
public:
	void Update_ECEF_init();
	void Geod2ECEF();
	void ECEF2ENU();
	
	double Pose_ENU_x() const;
	double Pose_ENU_y() const;
	double Pose_ENU_z() const;

	// subscriber callback functions

	void Gps_callback(const sensor_msgs::NavSatFix::ConstPtr&);
	void reset();
	void init_cond_callback(const std_msgs::Float32MultiArray::ConstPtr&);
};

} // end namespace {wmr_state_feedback}

#endif

