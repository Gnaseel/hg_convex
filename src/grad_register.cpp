#include<ros/ros.h>

#include<geometry_msgs/PoseStamped.h>
#include<sensor_msgs/PointCloud2.h>
#include<iostream>
#include<cmath>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

using namespace std;
using PointT = pcl::PointXYZI;
ros::Publisher clustered_pub;
ros::Publisher registered_pub;
class Box
{
    public:
    double minX, minY, minZ;
    double maxX, maxY, maxZ;
    int idx;
    Box()
    {
        minX = minY = minZ = 9999.0;
        maxX = maxY = maxZ = -9999.0;
        idx=-1;
    }
};

void ROIFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr output)
{
  pcl::CropBox<pcl::PointXYZI> cropFilter;
  cropFilter.setInputCloud(cloud);
  cropFilter.setMin(Eigen::Vector4f(0, -2, -0.7, 0)); // x, y, z, min (m)
  cropFilter.setMax(Eigen::Vector4f(18, 2, 0.8, 0));
  cropFilter.filter(*output);
}

void clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr output)
{
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new 	pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud (cloud);  //KdTree 생성

    vector<pcl::PointIndices> cluster_indices; 
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setInputCloud (cloud);
    ec.setClusterTolerance (1.0);
    ec.setMinClusterSize (80);     // 최소 포인트 수 
    ec.setMaxClusterSize (3000);  // 최대 포인트 수
    ec.setSearchMethod (tree);
    ec.extract (cluster_indices);

    int i = 0;
    pcl::PointCloud<PointT> TotalCloud;
    for(vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, ++i)
    {
      for(vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      {
        PointT pt = cloud->points[*pit];
        PointT pt2;

        pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
        pt2.intensity = (float)(i + 1);
        
        output->push_back(pt2);
      }
    }

}
geometry_msgs::PoseStamped objectPose;
int get_registered_index(list<Box>& boxes){
    if(boxes.size()==0 ){
        return 0;
    }
    double newX=0;
    double newY=0;
    double min_dist =5;
    int re_index = 0;
    for(auto box : boxes){
        double boxX = (box.minX+box.maxX)/2;
        double boxY = (box.minY+box.maxY)/2;
        double subX = (objectPose.pose.position.x - boxX)*(objectPose.pose.position.x - boxX);
        double subY = (objectPose.pose.position.y - boxY)*(objectPose.pose.position.y - boxY);
        double dist = sqrt(subX+subY);

        if(dist < min_dist){
            min_dist = dist;
            re_index = box.idx;
            newX=boxX;
            newY=boxY;
        }
    }
    cout<<"OBJ POSE = "<<objectPose.pose.position.x<<" "<<objectPose.pose.position.y<<endl;
    // cout<<"NEW POSE = "<<newX<<" "<<newY<<endl;

    objectPose.pose.position.x=newX;
    objectPose.pose.position.y=newY;
    cout<<"dist = "<<min_dist<<endl;
    system("clear");

    if(re_index== -1){
        cout<<"RE INDEX -1!!"<<endl;
        for(;;);
    }
    return re_index;
}
void updateBoundingBox(list<Box>& boxes, double x, double y, double z, int index)
{
    for(auto box : boxes){
        if(box.idx == index){
            if(x < box.minX) box.minX = x;
            if(y < box.minY) box.minY = y;
            if(z < box.minZ) box.minZ = z;
            if(x > box.maxX) box.maxX = x;
            if(y > box.maxY) box.maxY = y;
            if(z > box.maxZ) box.maxZ = z;
            return;
        }
    }
    Box box;
    box.minX = x;
    box.minY = y;
    box.minZ = z;
    box.maxX = x;
    box.maxY = y;
    box.maxZ = z;
    box.idx =index;
    boxes.push_back(box);
}
void register_publish(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int regi_vindex){
    
    pcl::PointCloud<pcl::PointXYZI> output;
    for(pcl::PointCloud<pcl::PointXYZI>::iterator it = cloud->begin(); it != cloud->end(); ++it){
        pcl::PointXYZI pt = *it;
        if(pt.intensity == regi_vindex){
            output.push_back(pt);
        }
    }
    sensor_msgs::PointCloud2 ros_output;
    pcl::toROSMsg(output, ros_output);
    ros_output.header.frame_id = "velodyne";

    registered_pub.publish(ros_output);
}
void pose_callback(const geometry_msgs::PoseStamped::ConstPtr msg){

    cout<<"POSE = "<<msg->pose.position.x<<" "<<msg->pose.position.y<<endl;
    objectPose.pose.position.x = msg->pose.position.x;
    objectPose.pose.position.y = msg->pose.position.y;
}
void velodyne_callback(const sensor_msgs::PointCloud2::ConstPtr msg){

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr output (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromROSMsg(*msg, *cloud);
    ROIFilter(cloud, cloud_roi);
    clustering(cloud_roi, output);


    
    
    pcl::PointCloud<pcl::PointXYZI>::iterator iter;

    list<Box>boxes;
    for(iter = output->begin();iter!=output->end();++iter)
    {
       updateBoundingBox(boxes, iter->x, iter->y, iter->z, iter->intensity);
    }
    int registered_index = get_registered_index(boxes);
    register_publish(output, registered_index);

    pcl::PCLPointCloud2 cloud_clustered;
    pcl::toPCLPointCloud2(*output, cloud_clustered);
    sensor_msgs::PointCloud2 ros_output;
    pcl_conversions::fromPCL(cloud_clustered, ros_output);

    // pcl::toROSMsg(TotalCloud, ros_output);
    ros_output.header.frame_id = "velodyne";
    clustered_pub.publish(ros_output);


}
int main(int argc, char** argv){


    ros::init(argc, argv,"register");
    ros::NodeHandle nh;


    ros::Subscriber pose_sub = nh.subscribe("/move_base_simple/goal",1, pose_callback);

	ros::Subscriber pc_sub = nh.subscribe("/velodyne_points",10,velodyne_callback);


    clustered_pub = nh.advertise<sensor_msgs::PointCloud2>("temptemp",10);
    registered_pub = nh.advertise<sensor_msgs::PointCloud2>("/target_registered",10);
    objectPose.pose.position.x=0;
objectPose.pose.position.y=0;
    ros::Rate l(80);

    while(ros::ok()){
        ros::spinOnce();
        l.sleep();
    }
    return 0;
}