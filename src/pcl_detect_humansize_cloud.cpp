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

class ExtractHumansizeCloud{
	public:
		ExtractHumansizeCloud();
		void cloud_cb(const sensor_msgs::PointCloud2Ptr& input);
	private:
		std::string robot_frame_;
		std::string pass_fieldname_;
		std::string save_file_path_;

		tf::TransformListner tf_listener_;
		ros::Publisher pub_;
		ros::subscriber sub_;

		bool is_save_;

		float voxel_resolution_;
		float outlier_mean_;
		float outlier_thresh_;
		float max_horizontal_height_;
		float min_horizontal_height_;
		float cluster_tolerance_;
		float min_cluster_size_;
		float max_cluster_size_;

		float min_target_width_;
		float max_target_width_;
		float min_target_depth_;
		float max_target_depth_;
		float min_target_height_;
		float max_target_height_;

		int intensity_threshold_;
};

ExtractHumansizeCloud::ExtractHumansizeCloud()
{
	pub_ = nh.advertise<sensor_msgs::PointCloud2> ("output_humansize_cloud", 1);
	sub_ = nh.subscribe ("hokuyo3d/hokuyo_cloud2", 0, cloud_cb);

	ros::NodeHandle private_nh("~");
	
	private_nh.param("is_save", is_save_, false);
	private_nh.param("robot_frame", robot_frame_, std::string("/base_link"));
	private_nh.param("voxel_gird_resolution", voxel_resolution_, 0.15);
	private_nh.param("outlier_removal_meanK", outiler_mean_, 10)
	private_nh.param("outlier_removal_StddevMulThresh", outlier_thresh_, 0.25)
	private_nh.param("horizon_field_name", pass_fieldname_,std::string("z"));
	private_nh.param("max_horizontal_height",max_horizontal_height_,0.0);
	private_nh.param("min_horizontal_height",min_horizontal_height_,-2.0);
	private_nh.param("cluster_tolerance", cluster_tolerance_, 0.2)
	private_nh.param("min_cluster_size",min_cluster_size_,10);
	private_nh.param("max_cluster_size",max_cluster_size_,1000000);
	private_nh.param("intensity_threshold", intensity_threshold_, 2000);
	private_nh.param("save_file_path", save_file_path_, "~/")
	private_nh.param("min_target_width", min_target_width_, 0.4);
	private_nh.param("max_target_width", max_target_width_, 1.2);
	private_nh.param("min_target_depth", min_target_depth_, 0.4);
	private_nh.param("max_target_depth", max_target_depth_, 1.2);
	private_nh.param("min_target_height", min_target_height_, 1.0);
	private_nh.param("max_target_height", max_target_height_, 2.0);

	ros::spin();
}

ExtractHumansizeCloud::cloud_cb(const sensor_msgs::PointCloud2Ptr& input)
{
	// Create a container for the data.
	sensor_msgs::PointCloud2 transformed_cloud;
	sensor_msgs::PointCloud2 output;
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr conv_input(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_boxel(new pcl::PointCloud<pcl::PointXYZI>());

	// Transform pointcloud from LIDAR fixed link to base_link
	tf::StampedTransform transform;
	try{
		tf_listener_->waitForTransform(target_link_name, input->header.frame_id, ros::Time(), ros::Duration(100.0));
	}catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	
	if(!pcl_ros::transformPointCloud(target_link_name, *input, 	transformed_cloud, *tf_listener_)){
		return;
	}

	//Conversion PointCloud2 intensity field name to PointXYZI intensity field name.
	transformed_cloud.fields[3].name = "intensity";
	pcl::fromROSMsg(transformed_cloud, *conv_input);

	// Voxel Grid
	pcl::VoxelGrid<pcl::PointXYZI> vg;
	vg.setInputCloud(conv_input);
	vg.setLeafSize(voxel_resolution_,voxel_resolution_,voxel_resolution_);
	vg.setDownsampleAllData(true);
	vg.filter(*cloud_boxel);

	// Satistical Outlier Removal
	pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
	sor.setInputCloud(cloud_boxel);
	sor.setMeanK(outlier_mean_);
	sor.setStddevMulThresh(outlier_thresh_);
	sor.filter(*cloud_boxel);

	//Pass Through Horizon
	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setFilterLimitsNegative(true);
	pass.setFilterFieldName(pass_fieldname_);
	pass.setFilterLimits(min_horizontal_height_,max_horizontal_height_);
	pass.filter(*cloud_boxel);

	//Make tree structure
	tree->setInputCloud(cloud_boxel);

	//Clustering
	std::vector<pcl::PointIndces> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
	ec.setClusterTolerance(cluster_tolerance_);
	ec.setMinClusterSize(min_cluster_size_);
	ec.setMaxClusterSize(max_cluster_size_);
	ec.setSearchMethod(tree)
	ec.setInputCloud(cloud_boxel);
	ec.extruct(cluster_indices);

	//Extract from size and intensity
	int now_cluster;
	for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_all_filtered(new pcl::PointCloud<pcl::PointXYZI>());
		float min_point[3];
		float max_point[3];
		pcl::PointXYZI min_pt, max_pt;
		int now_point;
		bool is_highIntensity = false;

		// Now iterator count
		now_cluster = it - cluster_indices.begin();

		//High Intensity Judge
		for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
		{
			if( cloud_boxel->points[*pit].intensity > intensity_threshold_){
				is_highIntensity = true;
				// If high intensity even one point, when build a flag
				break;
			}
		}
		if(is_highIntensity){
			for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
			{
				// Corresponding point copy to cloud_all_filterd
				cloud_all_filtered->points.push_back(cloud_boxel->points[*pit]);
				now_point = pit - it->indices.begin();
				// Measure size of high intensity index
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
		// Target size judge
		if(is_highIntensity){
			ROS_INFO_STREAM("Find include high intensity cloud");
			float target_size[3];
			
			// Calculation target size
			target_size[0] = fabs(max_pt.x - min_pt.x);
			target_size[1] = fabs(max_pt.y - min_pt.y);
			target_size[2] = fabs(max_pt.z - min_pt.z);

			if(    ((width < max_target_width_) && (width > min_target_width_))
				&& ((depth < max_target_depth_) && (depth > min_target_depth_))
				&& ((height < max_target_height_) && (height > min_target_height_))
				)
			{
				ROS_INFO_STREAM("Find Target");

				//Save Process
				cloud_all_filtered->width = 1;
				cloud_all_filtered->height = cloud_all_filtered->point.size();

				if(is_save_){
					std::string filename;
					std::stringstream filename_st;
					filename_st << input->header.stamp;
					filename.append(save_file_path_);
					filename.append(filename_st.str());
					filename.append(".pcd");
					pcl::io::savePCDFileASCII(filename, *cloud_all_filtered);
					ROS_INFO_STREAM("Save File to" << filename)
				}
				else{
					cloud_all_filtered->erase(cloud_all_filtered->begin()+1, cloud_all_filtered->end());
				}
				if(cloud_all_filtered->points.size() > 1){
					pcl::toROSMsg(*cloud_all_filtered, output);

					//Add header to output cloud
					output.header = input->header;
					output.header.frame_id = robot_frame_;

					pub_.publish(output);
				}
			}
		}
	}
}

int main(int argc, char** argv)
{
	//Initialize ROS
	ros::init(argc, argv, "pcl_detect_humansize_cloud");
	ros::NodeHandle nh;
	
	ExtractHumansizeCloud ehc;

	return 0;
}