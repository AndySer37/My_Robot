#include "icp_mapping.h"

mapping::mapping(){
	NodeHandle nh;
	map.reset(new PointCloud<PointXYZ>());
	map_process.reset(new PointCloud<PointXYZ>());
	pc_input.reset(new PointCloud<PointXYZ>());
	pc_filter.reset(new PointCloud<PointXYZ>());
	pc_target.reset(new PointCloud<PointXYZ>());
	result.reset(new PointCloud<PointXYZ>());

	downsample.setLeafSize (0.1f, 0.1f, 0.1f);
	//downsample_map.setLeafSize (0.001f, 0.001f, 0.001f);

	shape = visualization_msgs::Marker::LINE_STRIP;
	marker.header.frame_id = "/scan";
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.id = 0;
	marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	marker.type = shape;
	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;
	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 1.0f;
	marker.color.a = 1.0;

	initial_guess.setIdentity(4,4);
	pose.x = 0;
	pose.y = 0;
	pose.z = 0;
	icp.setMaxCorrespondenceDistance(100);
	icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(0.001);
	icp.setMaximumIterations(100); 
	count = 0;
	pc_process = nh.advertise<sensor_msgs::PointCloud2> ("/pc_process", 10);
	sub = nh.subscribe("/origin", 1, &mapping::process_node, this);

}
void mapping::process_node(const sensor_msgs::PointCloud2 msg){
	fromROSMsg (msg, *pc_input);
	count += 1;
	if (count == 1){
		*map += (*pc_input);
		return;
	}

	*pc_target = *map;

	// downsample.setInputCloud (pc_filter);
	// downsample.filter (*pc_filter);

	// icp
	// pcl::transformPointCloud (*pc_input, *pc_input, initial_guess);
	icp.setInputSource(pc_input);
	icp.setInputTarget(pc_target);
	cout << "Number PC of Source: "<< pc_input->size() << endl;
	cout << "Number PC of Target: "<< pc_target->size() << endl;
	icp.align(*result);
	initial_guess = icp.getFinalTransformation() * initial_guess;
	// pcl::transformPointCloud (*pc_input, *pc_input, initial_guess);

	// downsample.setInputCloud (pc_input);
	// downsample.filter (*pc_input);

	*map = (*pc_input);
	////



	// publish trajectory of car 
	// geometry_msgs::Point p;
	pose.x = initial_guess(0,3);
	pose.y = initial_guess(1,3);
	pose.z = initial_guess(2,3);
	marker.points.push_back(pose);
	marker.lifetime = ros::Duration();
	marker_pub.publish(marker);
	toROSMsg(*map, origin_map);
	origin_map.header.frame_id = "scan";
	pc_process.publish(origin_map);
}
int main(int argc, char** argv){
	init(argc, argv, "mapping");
	mapping mapping;
	spin();
	return 0;
}