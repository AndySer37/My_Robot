#include <iostream>
#include <math.h>
// Ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// Pcl load and ros
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/LaserScan.h>

using namespace ros;
using namespace pcl;
using namespace std;

class lasar_to_pcl{
  public:
	lasar_to_pcl();
	void process_node(sensor_msgs::LaserScan);    // sensor_msgs::PointCloud2
  private:
  	Subscriber sub;
	Publisher pc;
	PointCloud<PointXYZ>::Ptr map;
	sensor_msgs::PointCloud2 origin_map;
};

lasar_to_pcl::lasar_to_pcl(){
	NodeHandle nh;
	map.reset(new PointCloud<PointXYZ>());
	pc = nh.advertise<sensor_msgs::PointCloud2> ("/origin", 10);
	sub = nh.subscribe("/scan", 1, &lasar_to_pcl::process_node, this);
}
void lasar_to_pcl::process_node(const sensor_msgs::LaserScan msg){
	map->clear();
	for(int i = 0; i < msg.ranges.size(); i++){
		if (msg.ranges[i] > 0.1){
			float theta = i * msg.angle_increment;
			pcl::PointXYZ point;
			point.x = msg.ranges[i] * cos(theta);
			point.y = msg.ranges[i] * sin(theta);
			point.z = 0;
			map->points.push_back(point);
		}
		// cout << i << " : " << msg.ranges[i] << endl;
		// cout << "size : " << msg.ranges.size() << endl;
		// cout << msg.angle_min << " " << msg.angle_max << msg.angle_increment << endl;
	}
	toROSMsg(*map, origin_map);
	origin_map.header.frame_id = "scan";
	pc.publish(origin_map);
}
int main(int argc, char** argv){
	init(argc, argv, "lasar_to_pcl");
	lasar_to_pcl lasar_to_pcl;
	spin();
	return 0;
}