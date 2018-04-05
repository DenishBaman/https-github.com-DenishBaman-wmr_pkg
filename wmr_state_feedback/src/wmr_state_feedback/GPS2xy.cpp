/*
 > @Global to Local co-ordinate transformation defination
 > @file Gps2xy.cpp
*/

#include <wmr_state_feedback/GPS2xy.h>

using namespace wmr_state_feedback;

void GPS2xy::Update_ECEF_init()
{
  double L_n = semi_major/(std::sqrt(1-Eccen_sq*std::pow(std::sin(lat_init),2))); //Length of normal to the ellipsoid

	// X_ECEF_ini
	Pose_ECEF_init[0] = (L_n + alt_init) * std::cos(lat_init) * std::cos(lon_init);

	// Y_ECEF_ini 
	Pose_ECEF_init[1] = (L_n + alt_init) * std::cos(lat_init) * std::sin(lon_init);
	
	//Z_ECEF_ini 
	Pose_ECEF_init[2] = (L_n * (1 - Eccen_sq) + alt_init) * std::sin(lat_init);

}


void GPS2xy::Geod2ECEF()
{
 double  L_n = semi_major/(std::sqrt(1-Eccen_sq*std::pow(std::sin(lat),2))); //Length of normal to the ellipsoid

	// pose_ECEF in X co ordinate
	Pose_ECEF[0] = (L_n + alt) * std::cos(lat)*std::cos(lon);

        // pose_ECEF in Y co ordinate
        Pose_ECEF[1] = (L_n + alt) * std::cos(lat)*std::sin(lon);

        //pose_ECEF in Z co ordinate
        Pose_ECEF[2] = (L_n * (1 - Eccen_sq) + alt) * std::sin(lat);
}

void GPS2xy::ECEF2ENU()
{
	// Calculate difference between initial ECEF and Current ECEF co ordinates
    double delta_X = Pose_ECEF[0] - Pose_ECEF_init[0];
    double delta_Y = Pose_ECEF[1] - Pose_ECEF_init[1];
    double delta_Z = Pose_ECEF[2] - Pose_ECEF_init[2];



    // X_ENU
    Pose_ENU[0] = delta_Y*std::cos(lon) - delta_X*std::sin(lon);

    // Y_ENU
    Pose_ENU[1] = delta_Z*std::cos(lat) - delta_X*std::cos(lon)*std::sin(lat) -
                    delta_Y*std::sin(lon)*std::sin(lat);

    // Z_ENU
    Pose_ENU[2] = delta_Z*std::sin(lat) + delta_X*std::cos(lon)*std::cos(lat) +
                    delta_Y*std::cos(lat)*std::sin(lon);
}


// GPS_callback function
void GPS2xy::Gps_callback(const sensor_msgs::NavSatFix::ConstPtr& data)
{
    lat = data->latitude * M_PI/180.0;
    lon = data->longitude * M_PI/180.0;
    alt = data->altitude;
}

void GPS2xy::reset()
{
    lat_init = lat;
    lon_init = lon;
    alt_init = alt;

    // Update initial starting position in ECEF frame and store those values in pose_ECEF_init
    Update_ECEF_init();
}

void GPS2xy::init_cond_callback(const std_msgs::Float32MultiArray::ConstPtr& init_cond)
{
    lon_init = init_cond->data[0] * M_PI/180.0;
    lat_init = init_cond->data[1] * M_PI/180.0;
    alt_init = init_cond->data[2];
    Update_ECEF_init();
}

double GPS2xy::Pose_ENU_x() const // const member functions
{ return Pose_ENU[0]; }

double GPS2xy::Pose_ENU_y() const
{ return Pose_ENU[1]; }

double GPS2xy::Pose_ENU_z() const
{ return Pose_ENU[2];}	

