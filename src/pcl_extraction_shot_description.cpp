#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
#include <pcl/PCLPointCloud2.h>

void cloud_cb (const sensor_msgs::PointCloud2Ptr& input)
{
	// Create a container for the data.
	sensor_msgs::PointCloud2 output;
	pcl::PointCloud<pcl::PointXYZI>::Ptr conv_input(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::SHOT352>::Ptr descriptors(new pcl::PointCloud<pcl::SHOT352>());

	input->fields[3].name = "intensity";
	pcl::fromROSMsg(*input, *conv_input);

	pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(conv_input);
	normalEstimation.setRadiusSearch(0.03);
	normalEstimation.setSearchMethod(tree);
	normalEstimation.compute(*normals);

	pcl::SHOTEstimation<pcl::PointXYZI, pcl::Normal, pcl::SHOT352> shot;
	shot.setInputCloud(conv_input);
	shot.setInputNormals(normals);
	shot.setRadiusSearch(0.02);
	shot.compute(*descriptors);

	for(int i=0; i<descriptors->points.size(); i++){
		if(descriptors->points[i].descriptorSize()){
			for(int j=0; i<descriptors->points[i].descriptorSize(); i++){
				ROS_INFO_STREAM("Descriptors" << descriptors->points[i].descriptor[j]);
			}
		}
	}
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
