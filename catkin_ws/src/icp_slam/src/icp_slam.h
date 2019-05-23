#include <iostream>
#include <vector>
#include <Eigen/Dense>
// Ros
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
// Pcl load and ros
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
// Pcl icp
#include <pcl/registration/icp.h>
// Pcl passthrough
#include <pcl/filters/passthrough.h>
// Pcl outlier removal
#include <pcl/filters/statistical_outlier_removal.h>
// Pcl downsampling
#include <pcl/filters/voxel_grid.h>
// Pcl plane filter
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

using namespace ros;
using namespace pcl;
using namespace std;
const double PI  =3.141592653589793238463;

class mapping{
  public:
	mapping();
	void load_map();
	void process_node(const sensor_msgs::PointCloud2 msg);    // sensor_msgs::PointCloud2
	void add_map();
  private:
  	int len;
  	int count;
  	string path;
	Publisher pc_map_pub;
	Publisher pc_filter_pub;
	Publisher pc_map_loc_pub;
	Subscriber pc_sub;

	PointCloud<PointXYZ>::Ptr map;
	PointCloud<PointXYZ>::Ptr map_process;
	PointCloud<PointXYZ>::Ptr pc_input;
	PointCloud<PointXYZ>::Ptr pc_filter;
	PointCloud<PointXYZ>::Ptr pc_target;
	PointCloud<PointXYZ>::Ptr result;

	vector < PointCloud<PointXYZ> > pc_buffer;
	vector < Eigen::Matrix4f > pc_transform;

	IterativeClosestPoint<PointXYZ, PointXYZ> icp;
	PassThrough<PointXYZ> pass;
	StatisticalOutlierRemoval<PointXYZ> sor;
	VoxelGrid<PointXYZ> downsample;
	VoxelGrid<PointXYZ> downsample_map;

	Eigen::Matrix4f initial_guess;

	sensor_msgs::PointCloud2 origin_map;

	pcl::SACSegmentation<PointXYZ> seg;
	pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointIndices::Ptr inliers;
    pcl::ModelCoefficients::Ptr coefficients;

	Publisher marker_pub;
	uint32_t shape;
	visualization_msgs::Marker marker;
};