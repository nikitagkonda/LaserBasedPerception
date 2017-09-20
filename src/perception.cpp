#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "visualization_msgs/Marker.h"
#include <vector>
using namespace std;
ros::Publisher vel_pub;
ros::Publisher marker_pub;

struct Coordinate
{
	double x;	
	double y;
};

void sensorCallback(sensor_msgs::LaserScan msg)
{
	int i,flag=0,j=0;
	float speed, turn;
	/*vector<float> x;
	vector<float> y;*/
	vector<Coordinate> data;
	for(i=121;i<=240;i++)
	{
  		if(msg.ranges[i]<=1.7)
		{
			flag=1;
			break;
		}			
	}

	if(flag==1) {
		//ROS_INFO("I hit the obstacle");
		speed=-0.5;
		turn=2.0;
        }
	else{
	 	speed=2.0;
		turn=0.0;
	}
	geometry_msgs::Twist msg1;
	msg1.linear.x=speed;
	msg1.angular.z=turn;
       // vel_pub.publish(msg1);

	//ransac
	for(i=0;i<=360;i++)
	{
		if(msg.ranges[i]<3)
		{
			Coordinate dataObj;
			dataObj.x=msg.ranges[i]*cos(((i/2 - 90)*M_PI)/180);
			dataObj.y=msg.ranges[i]*sin(((i/2 - 90)*M_PI)/180);
			data.push_back(dataObj);
			/*data.x.push_back(msg.ranges[i]*cos(((i/2)*M_PI)/180));
			data.y.push_back(msg.ranges[i]*sin(((i/2)*M_PI)/180));*/
		}	
	}
	int k = 20; //number of iterations
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
		ROS_INFO("randoms: %f %f %f %f",x1,y1,x2,y2);

		//create a model from the 2 numbers
		m=(y2-y1)/(x2-x1);
		y0=m*x0-m*x1+y1;
		ROS_INFO("slope: %f",m);

		//check error
		vector<Coordinate> inliers;
		vector<int> inliersIndex;
		int inliers_count=0;
		for(j=0;j<data.size();j++)
		{
				num=abs(m+(-1)+(y1-m*x1));
				den=sqrt(m*m+1);
				d=num/den;
				ROS_INFO("slope: %f",d);
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

		if(inliers_count>bestInNum)
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
	line_list.ns = "perception";
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
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "perception");
	ros::NodeHandle n;
	vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	ros::Subscriber sub1 = n.subscribe<sensor_msgs::LaserScan>("base_scan", 10, sensorCallback);
	ros::spin();
	return 0;
}