#include "icp_slam.h"

mapping::mapping(){
	NodeHandle nh;
	map.reset(new PointCloud<PointXYZ>());
	map_process.reset(new PointCloud<PointXYZ>());
	pc_input.reset(new PointCloud<PointXYZ>());
	pc_filter.reset(new PointCloud<PointXYZ>());
	pc_target.reset(new PointCloud<PointXYZ>());
	result.reset(new PointCloud<PointXYZ>());


	// cov = {0.001, 0.0, 0.0, 0.0, 0.0, 0.0,\
	// 		0.0, 0.001, 0.0, 0.0, 0.0, 0.0,\
	// 		 0.0, 0.0, 0.001, 0.0, 0.0, 0.0,\
	// 		  0.0, 0.0, 0.0, 0.001, 0.0, 0.0,\
	// 		   0.0, 0.0, 0.0, 0.0, 0.001, 0.0,\
	// 		    0.0, 0.0, 0.0, 0.0, 0.0, 0.03};
	len = 12;
	count = 0;
	pc_buffer = vector< PointCloud<PointXYZ> >(0);
	pc_transform = vector< Eigen::Matrix4f >(0);

	coefficients.reset(new pcl::ModelCoefficients);
	inliers.reset(new pcl::PointIndices);

	shape = visualization_msgs::Marker::LINE_STRIP;
	marker.header.frame_id = "/base_link";
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.id = 0;
	marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	marker.type = shape;
	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 1.0f;
	marker.color.a = 1.0;

	initial_guess.setIdentity(4,4);

	icp.setMaxCorrespondenceDistance(100);
	icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(0.001);
	icp.setMaximumIterations(100); 

	pass.setFilterFieldName ("z");
	pass.setFilterLimits (-1, 1);

	sor.setMeanK (50);
	sor.setStddevMulThresh (0.4);
	downsample.setLeafSize (0.1f, 0.1f, 0.1f);
	downsample_map.setLeafSize (0.1, 0.1, 0.1);

	// Optional // plane filter 
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);  // pcl::SACMODEL_PLANE   SACMODEL_PERPENDICULAR_PLANE
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (50);								
	seg.setDistanceThreshold (0.5);
	Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0); 	// xy-plane 
	seg.setAxis(axis);
	seg.setEpsAngle(  50.0f * (PI/180.0f) );   				// Range degree of plane 
	extract.setNegative (true);

	pc_map_pub = nh.advertise<sensor_msgs::PointCloud2> ("/map_pc", 10);
	pc_filter_pub = nh.advertise<sensor_msgs::PointCloud2> ("/pc_filter", 10);
	pc_map_loc_pub = nh.advertise<nav_msgs::Odometry> ("/pc_loc_path", 10);
	pc_sub = nh.subscribe("/velodyne_points", 1, &mapping::process_node, this);
}
void mapping::process_node(const sensor_msgs::PointCloud2 msg){

	ros::Time begin = ros::Time::now();
	fromROSMsg (msg, *pc_input);
	if (count == 0){
		pc_buffer.push_back(*pc_input);
		pc_transform.push_back(initial_guess);	
		count ++;	
		return;
	}

	pc_target->clear();
	for (int i = 0; i < pc_buffer.size(); i++){
		*pc_target += pc_buffer[i];
	}
	toROSMsg(*pc_target, origin_map);
	origin_map.header.frame_id = "base_link";
	pc_filter_pub.publish(origin_map);


	// pre processing of input 
	// pass.setInputCloud (pc_input);
	// pass.filter (*pc_filter);
	// sor.setInputCloud (pc_input);
	// sor.filter (*pc_filter);

	downsample.setInputCloud (pc_input);
	downsample.filter (*pc_filter);
	downsample.setInputCloud (pc_target);
	downsample.filter (*pc_target);

	////

	// icp
	pcl::transformPointCloud (*pc_filter, *pc_filter, pc_transform.back());
	icp.setInputSource(pc_filter);
	icp.setInputTarget(pc_target);
	cout << "Number PC of Source: "<< pc_filter->size() << endl;
	cout << "Number PC of Target: "<< pc_target->size() << endl;
	icp.align(*result);
	initial_guess = icp.getFinalTransformation() * initial_guess;
	pcl::transformPointCloud (*pc_input, *pc_input, initial_guess);


	Eigen::Matrix3f mat3x3;
	mat3x3 << initial_guess(0,0), initial_guess(0,1), initial_guess(0,2),\
				initial_guess(1,0), initial_guess(1,1), initial_guess(1,2),\
				initial_guess(2,0), initial_guess(2,1), initial_guess(2,2);

	nav_msgs::Odometry nav;
	nav.header = msg.header;
	nav.child_frame_id = "base_link";
	Eigen::Quaternionf q1(mat3x3); 
	nav.pose.pose.position.x = initial_guess(0,3);
	nav.pose.pose.position.y = initial_guess(1,3);
	nav.pose.pose.position.z = initial_guess(2,3);
	nav.pose.pose.orientation.w = q1.w();
	nav.pose.pose.orientation.x = q1.x();
	nav.pose.pose.orientation.y = q1.y();
	nav.pose.pose.orientation.z = q1.z();
	for(int i = 0 ; i < 6; i++){
		nav.pose.covariance[i + i*6] = 0.001;
	}

	pc_map_loc_pub.publish(nav);
	////
	if (count > len){
		pc_buffer.erase(pc_buffer.begin());
		pc_transform.erase(pc_transform.begin());
	}
	pc_buffer.push_back(*pc_input);
	pc_transform.push_back(initial_guess);
	count ++;

	// publish trajectory of car 
	geometry_msgs::Point p;
	p.x = initial_guess(0,3);
	p.y = initial_guess(1,3);
	p.z = initial_guess(2,3);
	marker.points.push_back(p);
	marker.lifetime = ros::Duration();
	marker_pub.publish(marker);
	if(count % 15 == 0){
		add_map();
	}
	cout << "Spend time: " << ros::Time::now() - begin << endl;
}
void mapping::add_map(){
	// plane removal
	// seg.setInputCloud (pc_input);
	// seg.segment (*inliers, *coefficients);
	// extract.setInputCloud (pc_input);
	// extract.setIndices (inliers);
	// extract.filter (*pc_input);
	////		
	downsample_map.setInputCloud (pc_input);
	downsample_map.filter (*pc_input);	
	*map += (*pc_input);
	// downsample_map.setInputCloud (map);
	// downsample_map.filter (*map);	

	toROSMsg(*map, origin_map);
	origin_map.header.frame_id = "base_link";
	pc_map_pub.publish(origin_map);

	cout << "//////////////// Number PC of map: "<< map->size() << endl;		
}
int main(int argc, char** argv){
	init(argc, argv, "mapping");
	mapping mapping;
	spin();
	return 0;
}