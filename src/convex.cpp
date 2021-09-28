#include<ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <math.h>
#include <algorithm>
#define _USE_MATH_DEFINES

#include<sensor_msgs/PointCloud2.h>
#include<visualization_msgs/MarkerArray.h>
#include<visualization_msgs/Marker.h>
#include<geometry_msgs/PoseArray.h>
#include<std_msgs/UInt16.h>
#include<std_msgs/UInt8.h>
#include<std_msgs/Float32.h>

#include<iostream>


using namespace std;
#define	MAX_POINT	100000


typedef struct Point {
	double x;
	double y;
	bool operator<(const Point& p) {
		if (y != p.y) return x < p.x;
		else return y < p.y;
	}
	Point operator-(const Point& p) {
		Point ret = { x - p.x, y - p.y };
		return ret;
	}
} Point;


double cur_speed = 0;
ros::Publisher convex_pub, con_convex_pub, angle_pub, brake_pub, speed_pub;
void convec_publish(Point* p, int* stack, int size){
    visualization_msgs::MarkerArray arr;


	for(int i=0;i<size;i++){
		visualization_msgs::Marker marker;

		marker.header.frame_id = "/velodyne";
        marker.header.stamp  = ros::Time::now();
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.ns = "convex";
        marker.id = i;
        // marker.id =0;

        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.pose.position.z = 1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.05;
        marker.color.a = 1.0f;
        marker.color.b = 1.0f;
        marker.lifetime = ros::Duration(0.2);
		// marker.action = 0;

        geometry_msgs::Point sp;
        sp.x = p[stack[i]].x;
        sp.y = p[stack[i]].y;
        sp.z = 0;

        geometry_msgs::Point ep;
		if(size==i+1){
 			ep.x = p[stack[0]].x;
        	ep.y = p[stack[0]].y;
		}else{
 			ep.x = p[stack[i+1]].x;
        	ep.y = p[stack[i+1]].y;
		}
       
        ep.z = 0;

        marker.points.push_back(sp);
        marker.points.push_back(ep);
		arr.markers.push_back(marker);
	}
	convex_pub.publish(arr);

}
double ccw(Point &A, Point &B, Point &C) {
	Point BO = B - A;
	Point CO = C - A;
	return ((double)BO.x * (double)CO.y - (double)BO.y * (double)CO.x);
}
bool leftTurn(Point& A, Point& B, Point& C) { 
    
    return ccw(A, B, C) > 0; 
}

bool comp(Point &A, Point &B, Point *p)
{
	Point AO = A - p[0];
	Point BO = B - p[0];

	double outerProduct = ((double)AO.x * (double)BO.y - (double)AO.y * (double)BO.x);
	if (outerProduct != 0)
		return (outerProduct > 0);
	if (A.y != B.y)
		return A.y < B.y;
	return A.x < B.x;
}

void quickSortByAngle(int first, int last, Point *p)
{
	if (first >= last) return;

	int pivot = first;
	int i = first + 1;
	int j = last;

	while (i <= j)
	{
		while (comp(p[i], p[pivot], p) && i <= last) i++;
		while (!comp(p[j], p[pivot], p) && j > first) j--;

		if (i >= j) break;

		Point tmp = p[i];
		p[i] = p[j];
		p[j] = tmp;
	}

	Point tmp = p[j];
	p[j] = p[pivot];
	p[pivot] = tmp;

	quickSortByAngle(first, j - 1, p);
	quickSortByAngle(j + 1, last, p);
}

int convexHudouble(Point *p, int size)
{
    int stack[MAX_POINT];
	double minX = 1000000000, minY = 1000000000;
    int minIdx = 0;
	for (int i = 0; i < size; i++) {

		if (minY > p[i].y || (minY == p[i].y && minX > p[i].x))
		{
			minX = p[i].x;
			minY = p[i].y;
			minIdx = i;
		}
	}
	p[minIdx].x = p[0].x;
	p[minIdx].y = p[0].y;
	p[0].x = minX;
	p[0].y = minY;

	quickSortByAngle(1, size - 1, p);

	int idx = -1;
	stack[++idx] = 0;
	stack[++idx] = 1;

	int next = 2;
	while (next < size)
	{

		while ((idx + 1) >= 2)
		{
			int second = stack[idx--];
			int first = stack[idx];

			if (leftTurn(p[first], p[second], p[next]))
			{   
				stack[++idx] = second;
				break;
			}
		}
		stack[++idx] = next++;
        // cout<<"idx = "<<idx<<" next "<<next-1<<endl;

	}
	// convec_publish(p, stack, idx);
	geometry_msgs::PoseArray arr;


	for(int i=0;i<idx+1;i++){
		visualization_msgs::Marker marker;
		geometry_msgs::Pose pose;

		pose.position.x = p[stack[i]].x;
		pose.position.y = p[stack[i]].y;
		arr.poses.push_back(pose);

	}
	con_convex_pub.publish(arr);
    cout<<"INDEX SIZE = "<<idx<<" / "<<size<<endl;
	return idx + 1;
}


void speed_callback(const std_msgs::UInt16::ConstPtr& msg)
{
	cur_speed = msg -> data ;
}

void cons_callback(const geometry_msgs::PoseArray::ConstPtr msg){

    int size=0;
    Point p[MAX_POINT];
	for(int i=0;i<msg->poses.size();i++){
		double x =msg->poses[i].position.x; 
		double y =msg->poses[i].position.y; 
        p[size].x= x;
        p[size].y= y;
        size++;
	}
    convexHudouble(p, size);

}
int main(int argc, char** argv){


    ros::init(argc, argv,"convex");
    ros::NodeHandle nh;

	// ros::Subscriber pc_sub = nh.subscribe("/target_registered",10,registered_cadoubleback);
    ros::Subscriber speed_r_sub = nh.subscribe("/erp42/speed_r", 10, speed_callback);
	ros::Subscriber con_sub = nh.subscribe("/lane/cons",10,cons_callback);

	
    convex_pub = nh.advertise<visualization_msgs::MarkerArray>("/tempttt",100);
    con_convex_pub = nh.advertise<geometry_msgs::PoseArray>("/convex/cons",100);
    angle_pub = nh.advertise<std_msgs::Float32>("/control/angle",100);
    brake_pub = nh.advertise<std_msgs::UInt8>("/control/brake",100);
    speed_pub = nh.advertise<std_msgs::UInt16>("/control/accel",100);



    ros::Rate l(80);

    while(ros::ok()){
        // cout<<"asdfdsaf0"<<endl;
        ros::spinOnce();
        l.sleep();
    }
    return 0;
}