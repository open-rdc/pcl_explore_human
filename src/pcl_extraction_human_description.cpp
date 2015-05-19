#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/PCLPointCloud2.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <math.h>

#include <iostream>
#include <vector>
#include <algorithm>

using namespace Eigen;

void cloud_cb (const sensor_msgs::PointCloud2Ptr& input)
{
	// Create a container for the data.
	sensor_msgs::PointCloud2 output;
	pcl::PointCloud<pcl::PointXYZI>::Ptr conv_input(new pcl::PointCloud<pcl::PointXYZI>());
	
	input->fields[3].name = "intensity";
	pcl::fromROSMsg(*input, *conv_input);

	// Size of humansize cloud
	int cloud_size = conv_input->points.size();
	std::cout << "cloud_size: " << cloud_size << std::endl;

	// K nearest neighbor search
	pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
	kdtree.setInputCloud(conv_input);
	
	pcl::PointXYZI searchPoint;
	searchPoint.x = 0.0; searchPoint.y = 0.0; searchPoint.z = 0.0;

	int k = 10;
	float nearest_point_distance = 0;

	std::vector<int> pointIdxNKNSearch(k);
	std::vector<float> pointNKNSquiaredDistance(k);

	if( kdtree.nearestKSearch( searchPoint, k, pointIdxNKNSearch, pointNKNSquiaredDistance) > 0 ){
		nearest_point_distance = *min_element( pointNKNSquiaredDistance.begin(),  pointNKNSquiaredDistance.end() );
		std::cout << "min_distance: " << nearest_point_distance << std::endl;
	}

	// Calculate center of mass of humansize cloud
	Vector4f center_of_mass;
	pcl::compute3DCentroid(*conv_input, center_of_mass);
	MatrixXf convariance_matrix = MatrixXf::Zero(3,3);
	MatrixXf convariance_matrix_tmp = MatrixXf::Zero(3,3);
	MatrixXf moment_of_inertia_matrix_tmp = MatrixXf::Zero(3,3);
	MatrixXf moment_of_inertia_matrix = MatrixXf::Zero(3,3);
	Vector3f point_tmp;
	Vector3f point_from_center_of_mass;


	for(int j=0; j<conv_input->size(); j++){
		point_tmp << conv_input->points[j].x, conv_input->points[j].y, conv_input->points[j].z;
		//Calculate three dimentional convariance matrix
		point_from_center_of_mass <<
		 center_of_mass[0] - point_tmp[0],
		 center_of_mass[1] - point_tmp[1],
		 center_of_mass[2] - point_tmp[2];

		convariance_matrix_tmp += point_tmp * point_tmp.transpose();

		//Calculate three dimentional moment of inertia matrix
		moment_of_inertia_matrix_tmp << 
		powf(point_tmp[1],2.0f)+powf(point_tmp[2],2.0f),	-point_tmp[0]*point_tmp[1],							-point_tmp[0]*point_tmp[2],
		-point_tmp[0]*point_tmp[1],							powf(point_tmp[1],2.0f)+powf(point_tmp[2],2.0f),	-point_tmp[1]*point_tmp[2],
		-point_tmp[0]*point_tmp[2],							-point_tmp[1]*point_tmp[2],							powf(point_tmp[0],2.0f)+powf(point_tmp[1],2.0f);

		moment_of_inertia_matrix = moment_of_inertia_matrix + moment_of_inertia_matrix_tmp;
	}
	convariance_matrix = (1.0f/conv_input->points.size()) * convariance_matrix_tmp.array();
	
	for(int j=0;j<convariance_matrix.rows()*convariance_matrix.cols();j++){
		std::cout << convariance_matrix(j) << " ";
	}
	std::cout << std::endl;

	for(int j=0;j<moment_of_inertia_matrix.rows()*moment_of_inertia_matrix.cols();j++){
		std::cout << moment_of_inertia_matrix(j) << " ";
	}
	std::cout << std::endl;
}


int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "pcl_node");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("output_humansize_cloud", 0, cloud_cb);

	// Create a ROS publisher for the output point cloud
	//pub = nh.advertise<sensor_msgs::PointCloud2> ("output_humansize_cloud", 1);

	// Spin
	ros::spin ();
}
