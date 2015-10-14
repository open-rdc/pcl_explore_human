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
	bool is_save = false;
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
	
	ROS_INFO_STREAM( "is SACSegmentation" << sacSegmentFlag );

	if(!sacSegmentFlag){
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
	tree->setInputCloud( cloud_boxel );
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
	ec.setClusterTolerance(0.2);
	ec.setMinClusterSize(10);
	ec.setMaxClusterSize(1000000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_boxel);
	ec.extract(cluster_indices);
	int j=0;
	/*---for Debug Visualize(intensity coloring)---*/
	int size = cluster_indices.size();
	int color_max = 15000;
	/*---------------------------------------------*/
	int now_cluster;
	for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_all_filtered(new pcl::PointCloud<pcl::PointXYZI>());
		//Now iterator count
		now_cluster = it - cluster_indices.begin();
		float min_point[3];
		float max_point[3];
		pcl::PointXYZI min_pt,max_pt;
		int now_point;
		bool is_highIntensity = false;
		//High intensity judge
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
		{
			if( cloud_boxel -> points[*pit].intensity > 2000 ){
				is_highIntensity = true;
				break;
			}
		}
		if(is_highIntensity){
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			{
				cloud_all_filtered->points.push_back(cloud_boxel->points[*pit]);
				now_point = pit - it->indices.begin();
				/*---measure size of high intensity index---*/
				if( now_point == 0 ){
					min_pt = cloud_boxel -> points[*pit];
					max_pt = cloud_boxel -> points[*pit];
				}
				if( min_pt.x > cloud_boxel -> points[*pit].x ){
					min_pt.x = cloud_boxel -> points[*pit].x;
				}
				else if( max_pt.x < cloud_boxel -> points[*pit].x ){
					max_pt.x = cloud_boxel -> points[*pit].x;
				}
				if( min_pt.y > cloud_boxel -> points[*pit].y ){
					min_pt.y = cloud_boxel -> points[*pit].y;
				}
				else if( max_pt.y < cloud_boxel -> points[*pit].y ){
					max_pt.y = cloud_boxel -> points[*pit].y;
				}
				if( min_pt.z > cloud_boxel -> points[*pit].z ){
					min_pt.z = cloud_boxel -> points[*pit].z;
				}
				else if( max_pt.z < cloud_boxel -> points[*pit].z ){
					max_pt.z = cloud_boxel -> points[*pit].z;
				}
			}
		}
		if( is_highIntensity ){
			std::cout<<"HighIntensity"<<std::endl;

			std::string file_path = "/home/kazuya/pcd/";
			int file_cnt = 0;
			float width,depth,height;
			width = fabs(max_pt.x - min_pt.x);
			depth = fabs(max_pt.y - min_pt.y);
			height = fabs(max_pt.z - min_pt.z);
			std::cout << "size:" << width << "," << depth << "," << height << "," << std::endl;
			if( ((width < 1.2) && (width > 0.4)) && ((depth < 1.2) && (depth > 0.4)) && ((height < 2.0) && (height > 1.0)) ){
				std::cout << "Find Target" << std::endl;

				//Save process
				cloud_all_filtered -> width = 1;
				cloud_all_filtered -> height = cloud_all_filtered -> points.size();

				if(is_save){
					std::stringstream file_name_st;
					std::string file_name;
					file_name_st << input->header.stamp;
					file_name.append(file_path);
					file_name.append(file_name_st.str());
					file_name.append(".pcd");
					std::cout << file_name <<std::endl;
					pcl::io::savePCDFileASCII(file_name, *cloud_all_filtered);
					file_cnt++;
				}
			}
			else{
				//if cloud is not target, leaving only one point to erase all.
				cloud_all_filtered->erase(cloud_all_filtered->begin()+1,cloud_all_filtered->end());
			}
			if( cloud_all_filtered->points.size() > 1 ){
				pcl::toROSMsg(*cloud_all_filtered, output);
				// pcl::toROSMsg(*cloud_boxel, output);

				//add header to output cloud.
				output.header = input -> header;
				output.header.frame_id = "base_link";

				// Publish the data.
				pub.publish (output);
			}
		}
	}

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
