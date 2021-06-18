#define _USE_MATH_DEFINES

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <fstream>	
#include <cassert>
#include <math.h>
#include <vector>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "icars_msgs/Gps.h"

using namespace std;
#define debug 1

const double pi = M_PI;
int flag = 0;

typedef struct VehicleState{
	double x = 0; //current x in meter
	double y = 0; //current y in meter
	double yaw_deg = 0; //current yaw in degree
	double yaw_rad = yaw_deg*pi/180; //current yaw in radian
	double speed = 0; //current speed in m/s
} State;
State state;

typedef struct LocationPoint{
	double x = 0; //m
	double y = 0; //m
} Point;
vector<Point> path;

typedef struct gpsData {
	double gpsLongitude = 0; //degree
	double gpsLatitude = 0;	//degree
} gpsPoint;



void update_speed(const std_msgs::Float64::ConstPtr& msg)
{
	//comment
	state.speed = msg->data;
	if (debug) {ROS_INFO("update current car speed: [%f]", state.speed);}
}

void update_location(const icars_msgs::Gps::ConstPtr& msg)
{
	//comment
	state.x = msg->x;
	state.y = msg->y;
	if (debug) {ROS_INFO("update current car location : [%f, %f]", state.x, state.y);}
}

void update_yaw(const std_msgs::Float64::ConstPtr& msg)
{
	//comment
	state.yaw_deg = msg->data;
	//state.yaw = 90.0-state.yaw;
	if (debug) {ROS_INFO("update current car yaw: [%f]", state.yaw_deg);}
	flag = 1;
}

double TM_coordinate(const double gpsLatitude, const double gpsLongitude)
{
	//Direct transformation formulae
	double k0 = 0.9996; //scale factor of point scale
	const a = 6378137;	//meter equatorial radius 
	double x = 0.5*k0*a*log2((1+sin(gpsLongitude)*cos(gpsLatitude))/(1-sin(gpsLongitude)*cos(gpsLatitude)));
	double y = k0*a*atan(tan(gpsLatitude)/cos(gpsLongitude));
	return x, y
}

void load_path(const string& path_file, vector<Point>& path)
{
	//comment
	ifstream in;
	in.open(path_file, ios::in);
	if(!in.is_open())
	{
		cout <<"Error:open path file failed!" <<endl;
		return;
	}

	string strOne;
	while(getline(in,strOne))
	{
		cout << strOne <<endl;
		stringstream ss;
		ss << strOne;
		Point p;
		ss >> p.x >> p.y;
		path.push_back(p);
		//cout <<"=================="<<endl;
	}

	cout << "load path succeed!" << endl;
}

double pure_pursuit_control(const State& s, const vector<Point>& path)
{
	//comment
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controller");
	ros::NodeHandle n;
	ros::Rate loop_rate(20);

	ros::Subscriber sub_speed = n.subscribe("speed_topic", 1000, update_speed);
	//ros::Subscriber sub_location = n.subscribe("location_topic", 1000, update_location);
	ros::Subscriber sub_yaw = n.subscribe("yaw_topic", 1000, update_yaw);

	std_msgs::Float64 steerAngle;
	ros::Publisher pub_steerAngle = n.advertise<std_msgs::Float64>("steerAngle_topic",5);
	
	
	string path_file = "./src/pure_pursuit_test/src/write.txt";

	load_path(path_file, path);
	if (path.size() ==0)
	{
		printf("Error: load path failed!");
		exit(1);
	}

	//cout<<"Toan"<< path.size();

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
		if (flag) break;
	}

	while (ros::ok())
	{
		
		//cout << "speed: "<<state.speed<<endl;
		cout << "yaw: "<<state.yaw_deg<<endl;
		//double alpha = state.speed + state.yaw_deg;
		//cout<<"alpha: "<<alpha<<endl;
		steerAngle.data = state.yaw_deg;
		//cout<<"steerAngle: "<<steerAngle.data<<endl;
		if (debug) {ROS_INFO("calculated steer angle: %f", steerAngle.data);}
		pub_steerAngle.publish(steerAngle);

		cout <<"======================"<<endl;
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;

}

