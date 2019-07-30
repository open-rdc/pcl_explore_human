#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <iostream>
#include <string>
#include <sstream>


void pass_through_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,bool LimitsNegative,const std::string &FieldName,float min_Limits,float max_Limits){
  pcl::PassThrough<pcl::PointXYZI> pass_x;
	pass_x.setInputCloud(cloud);
	pass_x.setFilterLimitsNegative(LimitsNegative);
	pass_x.setFilterFieldName(FieldName);
	pass_x.setFilterLimits(min_Limits,max_Limits);
	pass_x.filter(*cloud);
};

class ExtractHumansizeCloud{
  public:
    ExtractHumansizeCloud(){

      ros::NodeHandle private_nh_("~");
     
      private_nh_.getParam("robot_frame",robot_frame_);
      private_nh_.getParam("lrf_frame",lrf_frame_);
      private_nh_.getParam("voxel_resolution",voxel_resolution_);
      private_nh_.getParam("outlier_mean",outlier_mean_);
      private_nh_.getParam("outlier_threshold",outlier_threshold_);
      private_nh_.getParam("search_range",search_range_);
      private_nh_.getParam("min_horizontal_height",min_horizontal_height_);
      private_nh_.getParam("max_horizontal_height",max_horizontal_height_);
      private_nh_.getParam("cluster_tolerance",cluster_tolerance_);
      private_nh_.getParam("min_cluster_size",min_cluster_size_);
      private_nh_.getParam("max_cluster_size",max_cluster_size_);
      private_nh_.getParam("min_target_width",min_target_width_);
      private_nh_.getParam("max_target_width",max_target_width_);
      private_nh_.getParam("min_target_depth",min_target_depth_);
      private_nh_.getParam("max_target_depth",max_target_depth_);
      private_nh_.getParam("min_target_height",min_target_height_);
      private_nh_.getParam("max_target_height",max_target_height_);
      private_nh_.getParam("save_to_pcd", save_to_pcd_);
      private_nh_.getParam("save_file_path",save_file_path_);

      sub_ = nh_.subscribe<sensor_msgs::PointCloud2> (lrf_frame_, 1, &ExtractHumansizeCloud::cloud_cb,this);
      pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("output_filter_cloud", 1);
      pub2_ = nh_.advertise<sensor_msgs::PointCloud2> ("output_humansize_cloud", 1);
      tf_listener_ = new tf::TransformListener();

    }
  
  private:

    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Publisher pub2_;

    tf::TransformListener *tf_listener_;
    
    std::string robot_frame_;
    std::string lrf_frame_;

    double voxel_resolution_;
		double outlier_mean_;
		double outlier_threshold_;
    double search_range_;
		double min_horizontal_height_;
		double max_horizontal_height_;
		double cluster_tolerance_;
    int min_cluster_size_;
		int max_cluster_size_;

    double min_target_width_;
    double max_target_width_;
    double min_target_depth_;
    double max_target_depth_;
    double min_target_height_;
    double max_target_height_;

    bool save_to_pcd_;
    std::string save_file_path_;
    

};

void 
ExtractHumansizeCloud::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  //transform cloud
  tf::StampedTransform transform;
  sensor_msgs::PointCloud2 transformed_cloud;

	try{
		tf_listener_->waitForTransform(robot_frame_, cloud_msg->header.frame_id, ros::Time(0), ros::Duration(100.0));
	}catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	
	if(!pcl_ros::transformPointCloud(robot_frame_, *cloud_msg, transformed_cloud, *tf_listener_)){
		return;
	}
  
  //pcl to rosmsg
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg (transformed_cloud, *cloud);
  
  //filter container
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_boxel(new pcl::PointCloud<pcl::PointXYZI>());

  // Voxel Grid
	pcl::VoxelGrid<pcl::PointXYZI> vg;
	vg.setInputCloud(cloud);
	vg.setLeafSize(voxel_resolution_,voxel_resolution_,voxel_resolution_);
	vg.setDownsampleAllData(true);
	vg.filter(*cloud_boxel);

  // Satistical Outlier Removal
	pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
	sor.setInputCloud(cloud_boxel);
	sor.setMeanK(outlier_mean_);
	sor.setStddevMulThresh(outlier_threshold_);
	sor.filter(*cloud_boxel);

  //Pass through filter
  pass_through_filter(cloud_boxel,false,"x",-search_range_,search_range_);
  pass_through_filter(cloud_boxel,false,"y",-search_range_,search_range_);
  pass_through_filter(cloud_boxel,true,"z",min_horizontal_height_,max_horizontal_height_);

  //Pass Through Horizon X
  /* 
	pcl::PassThrough<pcl::PointXYZI> pass_x;
	pass_x.setInputCloud(cloud_boxel);
	pass_x.setFilterLimitsNegative(false);
	pass_x.setFilterFieldName("x");
	pass_x.setFilterLimits(-search_range_,search_range_);
	pass_x.filter(*cloud_boxel);

  //Pass Through Horizon Y
	pcl::PassThrough<pcl::PointXYZI> pass_y;
	pass_y.setInputCloud(cloud_boxel);
	pass_y.setFilterLimitsNegative(false);
	pass_y.setFilterFieldName("y");
	pass_y.setFilterLimits(-search_range_,search_range_);
	pass_y.filter(*cloud_boxel);
  
  //Pass Through Horizon Z
	pcl::PassThrough<pcl::PointXYZI> pass_z;
	pass_z.setInputCloud(cloud_boxel);
	pass_z.setFilterLimitsNegative(true);
	pass_z.setFilterFieldName("z");
	pass_z.setFilterLimits(-20.0,0.05);
	pass_z.filter(*cloud_boxel);
*/
  // Convert to ROS data type
  sensor_msgs::PointCloud2 output_filter_cloud;
  pcl::toROSMsg(*cloud_boxel, output_filter_cloud);

  // Publish the data
  pub_.publish (output_filter_cloud);

	// Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud (cloud_boxel);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance (cluster_tolerance_); // 20cm
  ec.setMinClusterSize (min_cluster_size_);
  ec.setMaxClusterSize (max_cluster_size_);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_boxel);
  ec.extract (cluster_indices);

  int j=0;

  pcl::PCDWriter writer;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointXYZI min_pt, max_pt;

    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    {
      cloud_cluster->points.push_back (cloud_boxel->points[*pit]); //*

      if( pit == it->indices.begin()){
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

      double target_size[3];
      // Calculation target size
		  target_size[0] = fabs(max_pt.x - min_pt.x);
		  target_size[1] = fabs(max_pt.y - min_pt.y);
		  target_size[2] = fabs(max_pt.z - min_pt.z);

      //ROS_INFO("%lf %lf %lf",target_size[0],target_size[1],target_size[2]);

      if(    ((target_size[0] < max_target_width_)  && (target_size[0] > min_target_width_))
			    && ((target_size[1] < max_target_depth_)  && (target_size[1] > min_target_depth_))
			    && ((target_size[2] < max_target_height_) && (target_size[2] > min_target_height_))
			)
		  {

        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        if(save_to_pcd_ == true){
          //save process
          std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
          std::stringstream ss;
          ss <<save_file_path_<< "cloud_cluster_" << j << ".pcd";
          writer.write<pcl::PointXYZI> (ss.str (), *cloud_cluster, false); //*
          j++;
        }

        if(cloud_cluster->points.size() > 1){
          sensor_msgs::PointCloud2 output_humansize_cloud;
				  pcl::toROSMsg(*cloud_cluster, output_humansize_cloud);

				  //Add header to output cloud
				  output_humansize_cloud.header = cloud_msg->header;
				  output_humansize_cloud.header.frame_id = robot_frame_;

				  pub2_.publish(output_humansize_cloud);
        }
      }
    }
  }
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_detect_humansize_cloud");

  ExtractHumansizeCloud ex;
  
  // Spin
  ros::spin ();
}
