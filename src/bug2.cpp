#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "visualization_msgs/Marker.h"
#include <vector>
#include <math.h>
#include <ros/console.h>
using namespace std;
ros::Publisher vel_pub;
ros::Publisher marker_pub;
double robot_x;
double robot_y;
double robot_angle;
const int goalseek=0;
const int wallfollow=1;
int state = goalseek;

struct Coordinate
{
	double x;	
	double y;
};

void baseCallback(nav_msgs::Odometry msg)
{
	std::cout << "Inside callback!" << std::endl;
	robot_x=msg.pose.pose.position.x;
	robot_y=msg.pose.pose.position.y;
	double w=msg.pose.pose.orientation.w;
	double z=msg.pose.pose.orientation.z;
	robot_angle=atan((2*w*z)/(1-(2*z*z)));
	ROS_INFO("Robot angle: %f",robot_angle);
}

void sensorCallback(sensor_msgs::LaserScan msg)
{
	int i,flag=0,j=0,sen_flag=1;
	float speed, turn;
	vector<Coordinate> data;

	//ransac
	for(i=0;i<=360;i++)
	{
		if(msg.ranges[i]<3)
		{
			Coordinate dataObj;
			dataObj.x=msg.ranges[i]*cos(((i/2 - 90)*M_PI)/180);
			dataObj.y=msg.ranges[i]*sin(((i/2 - 90)*M_PI)/180);
			data.push_back(dataObj);
		}	
	}
	int k = 50; //number of iterations
	int t = 50; //threshold
	float e = 0.1; //error
	float x1,y1,x2,y2,m,y0,x0,num,den,d;
	int rand_index_1, rand_index_2;
	
	int bestInNum=0,l;
	Coordinate bestParameter1;
	Coordinate bestParameter2;
	if(data.size()>4)
	{
	for(i=0;i<k;i++)
	{
		//ROS_INFO("size of x: %d",x.size());
		//select 2 random numbers
		rand_index_1 = rand() % data.size();
		x1=data[rand_index_1].x;
		y1=data[rand_index_1].y;
		do{
		rand_index_2=rand() % data.size();
		}while(rand_index_2==rand_index_1);
		x2=data[rand_index_2].x;
		y2=data[rand_index_2].y;
		//ROS_INFO("randoms: %f %f %f %f",x1,y1,x2,y2);

		//create a model from the 2 numbers
		m=(y2-y1)/(x2-x1);
		y0=m*x0-m*x1+y1;
		//ROS_INFO("slope: %f",m);

		//check error
		vector<Coordinate> inliers;
		vector<int> inliersIndex;
		int inliers_count=0;
		for(j=0;j<data.size();j++)
		{
				num=abs(m+(-1)+(y1-m*x1));
				den=sqrt(m*m+1);
				d=num/den;
				//ROS_INFO("slope: %f",d);
				if(d<=e)
				{
					Coordinate inlierObj;
					inlierObj.x=data[j].x;
					inlierObj.y=data[j].y;
					inliers.push_back(inlierObj);
					inliersIndex.push_back(j);
					inliers_count++;
				}
			
		}

		if(inliers_count>t)
		//regression
		{
			//better model
			bestInNum=inliers_count;
			//bestfit
			double max=0,dist;
			for(j=0;j < inliers.size()-1;j++)
			{
				for(l=j+1; l<inliers.size();l++)
				{
					dist=sqrt(pow((inliers[j].x - inliers[l].x),2) + pow((inliers[j].y - inliers[l].y),2));
					if(dist>max)
					{
						max=dist;
						bestParameter1.x= inliers[j].x;
						bestParameter1.y= inliers[j].y;
						bestParameter2.x= inliers[l].x;
						bestParameter2.y= inliers[l].y;	
					}	
				}
			}
			//erase
			sort(inliersIndex.begin(),inliersIndex.end());
			for(j=inliersIndex.size()-1; j>=0;j--)
			{
				data.erase(data.begin()+inliersIndex[j]);
			}
		}
		
		if(data.size()<4)
			break;
		
	}
	visualization_msgs::Marker line_list;
	line_list.header.frame_id = "/base_link";
	line_list.header.stamp = ros::Time::now();
	line_list.ns = "bug2";
	line_list.action = visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w=1;
   	line_list.id = 0;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.scale.x = 0.1;
    	line_list.scale.y = 0.1;
	line_list.color.g = 1.0f;
    	line_list.color.a = 1.0;
	geometry_msgs::Point p;
	p.x = bestParameter1.x;
	p.y = bestParameter1.y;
	line_list.points.push_back(p);
	p.x = bestParameter2.x;
	p.y = bestParameter2.y;
	line_list.points.push_back(p);
	marker_pub.publish(line_list);
	}

	//bug2

	double goal_x=4.5;
	double goal_y=9.0;
	double goal_angle=atan((goal_y-robot_y)/(goal_x-robot_x))-robot_angle;
	double goal_distance=sqrt(pow((goal_x-robot_x),2)+pow((goal_y-robot_y),2));
	ROS_INFO("Goal angle: %f",goal_angle);
	ROS_INFO("Robot x: %f",robot_x);
	for(i=121;i<=240;i++)
	{
  		if(msg.ranges[i]<=1.0)
		{
			flag=1;
			break;
		}			
	}
		
	if(state==goalseek)
	{
	if(flag==1)
		state=wallfollow;
	else{
	while(abs(goal_angle-robot_angle)>0.1743)
	//rotate robot
	{
		geometry_msgs::Twist msg1;
		msg1.linear.x=0;
		msg1.angular.z=-0.3;
		vel_pub.publish(msg1);
		return;
	}
		geometry_msgs::Twist msg1;
		msg1.linear.x=2.0;
		msg1.angular.z=0;
		vel_pub.publish(msg1);
	
	}}
	
	if(state==wallfollow)
	{
		double ransac_angle=atan((bestParameter2.y-bestParameter1.y)/(bestParameter2.x-bestParameter1.x));
		ROS_INFO("ransac angle %f",ransac_angle);
		while(abs(ransac_angle)>=0.3)
		{
			geometry_msgs::Twist msg1;
			msg1.angular.z=-0.3;
			vel_pub.publish(msg1);
			return;
		}
		geometry_msgs::Twist msg2;
		msg2.linear.x=2.0;
		vel_pub.publish(msg2);
	}
}


int main(int argc, char **argv)
{
	std::cout << "Hello World!" << std::endl;
	ros::init(argc, argv, "bug2");
	ros::NodeHandle n;
	vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	ros::Subscriber sub1 = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth", 10, baseCallback);
	ros::Subscriber sub2 = n.subscribe<sensor_msgs::LaserScan>("base_scan", 10, sensorCallback);

	ros::spin();
	return 0;
}
