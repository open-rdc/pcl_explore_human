// +ROS specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/conditional_euclidean_clustering.hpp>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/PCLPointCloud2.h>

#include <sys/time.h>

ros::Publisher pub;
tf::TransformListener *tf_listener;

void cloud_cb (const sensor_msgs::PointCloud2Ptr& input)
{
	// Create a container for the data.
	sensor_msgs::PointCloud2 transformed_cloud;
	sensor_msgs::PointCloud2 output;
	pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr conv_input(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_boxel(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_outlier_filtered(new pcl::PointCloud<pcl::PointXYZI>());

	/*--- for debug ---*/
	bool sacSegmentFlag = false;
	//bool is_save = false;

	struct timeval s, f;
	//Get start time
	gettimeofday(&s, NULL);
	/*---           ---*/
	
	// Transform pointcloud from LIDAR fixed link to base_link
	std::string target_link_name = "/base_link";
	tf::StampedTransform transform;
	try{
		tf_listener->waitForTransform(target_link_name, input->header.frame_id, ros::Time(), ros::Duration(100.0));
	}catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	
	if(!pcl_ros::transformPointCloud(target_link_name, *input, 	transformed_cloud, *tf_listener)){
		return;
	}
	
	//Conversion PointCloud2 intensity field name to PointXYZI intensity field name.
	transformed_cloud.fields[3].name = "intensity";
	pcl::fromROSMsg(transformed_cloud, *conv_input);

	pcl::VoxelGrid<pcl::PointXYZI> vg;
	vg.setInputCloud(conv_input);
	vg.setLeafSize(0.15,0.15,0.15);
	vg.setDownsampleAllData(true);
	vg.filter(*cloud_boxel);

	pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
	sor.setInputCloud(cloud_boxel);
	sor.setMeanK(10);
	sor.setStddevMulThresh(0.25);
	sor.filter(*cloud_boxel);



/*if(!sacSegmentFlag){
		pcl::PassThrough<pcl::PointXYZI> pass;
		pass.setFilterLimitsNegative (true);
		pass.setInputCloud(cloud_boxel);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(-2.0,0.0);
		pass.filter(*cloud_boxel);
	}
	else{
		//Plane segmentation
		pcl::SACSegmentation<pcl::PointXYZI> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setAxis(Eigen::Vector3f(0,0,1));
		seg.setEpsAngle (20);
		//Iteration count
		seg.setMaxIterations(1000);
		seg.setDistanceThreshold(0.01);

		//Plane Extract
		pcl::ExtractIndices<pcl::PointXYZI> extract;
		int i=0, nr_points = (int) cloud_boxel->points.size();
		while( cloud_boxel->points.size() > 0.3 * nr_points )
		{
			seg.setInputCloud(cloud_boxel);
			seg.segment(*inliers, *coefficients);
			if( inliers->indices.size() == 0)
			{
				std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
				break;
			}
			extract.setInputCloud(cloud_boxel);
			extract.setIndices(inliers);
			extract.setNegative(true);
			extract.filter(*cloud_boxel);
		}
	}
	*/


/*	pcl::PassThrough<pcl::PointXYZI> passz;
	passz.setFilterLimitsNegative (true);
	passz.setInputCloud(cloud_boxel);
	passz.setFilterFieldName("z");
	passz.setFilterLimits(-0.0,0.0);
	passz.filter(*cloud_boxel);
*/





//wall
	
	pcl::PassThrough<pcl::PointXYZI> passx;
	passx.setFilterLimitsNegative (true);
	passx.setInputCloud(conv_input);
	passx.setFilterFieldName("x");
	passx.setFilterLimits(8.0,50.0);
	passx.filter(*cloud_boxel);

	pcl::PassThrough<pcl::PointXYZI> passy;
	passy.setFilterLimitsNegative (true);
	passy.setInputCloud(cloud_boxel);
	passy.setFilterFieldName("y");
	passy.setFilterLimits(8.0,50.0);
	passy.filter(*cloud_boxel);

	pcl::PassThrough<pcl::PointXYZI> pass2y;
	pass2y.setFilterLimitsNegative (true);
	pass2y.setInputCloud(cloud_boxel);
	pass2y.setFilterFieldName("y");
	pass2y.setFilterLimits(-50.0, -8.0);
	pass2y.filter(*cloud_boxel);






/*
	pcl::PassThrough<pcl::PointXYZI> passy;
	passy.setFilterLimitsNegative (true);
	passy.setInputCloud(cloud_boxel);
	passy.setFilterFieldName("y");
	passy.setFilterLimits(-0.0,5.0);
	passy.filter(*cloud_boxel);
	*/

	/*pcl::PassThrough<pcl::PointXYZI> right_test;
	right_test.setFilterLimitsNegative (true);
	right_test.setInputCloud(cloud_boxel);
	right_test.setFilterFieldName("y");
	right_test.setFilterLimits(-50.0 , 1-2.0);
	right_test.filter(*cloud_boxel);

	pcl::PassThrough<pcl::PointXYZI> left_test;
	left_test.setFilterLimitsNegative (true);
	left_test.setInputCloud(cloud_boxel);
	left_test.setFilterFieldName("y");
	left_test.setFilterLimits(1+2.0 , 50.0);
	left_test.filter(*cloud_boxel);

	pcl::PassThrough<pcl::PointXYZI> above_test;
	above_test.setFilterLimitsNegative (true);
	above_test.setInputCloud(cloud_boxel);
	above_test.setFilterFieldName("z");
	above_test.setFilterLimits(1+2.0,50.0);
	above_test.filter(*cloud_boxel);
*/




	


	pcl::toROSMsg(*cloud_boxel, output);
	// pcl::toROSMsg(*cloud_boxel, output);

	//add header to output cloud.
	output.header = input -> header;
	output.header.frame_id = "base_link";

	// Publish the data.
	pub.publish (output);
			


	//get finish time
	gettimeofday(&f, NULL);
	ROS_INFO_STREAM( "Compute time:" << (f.tv_sec - s.tv_sec) * 1000 + (f.tv_usec - s.tv_usec) / 1000 << "ms" );
}


int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "pcl_node");
	ros::NodeHandle nh;

	tf_listener = new tf::TransformListener();

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("hokuyo3d/hokuyo_cloud2", 0, cloud_cb);

	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2> ("output_humansize_cloud", 1);

	// Spin
	ros::spin ();
}
