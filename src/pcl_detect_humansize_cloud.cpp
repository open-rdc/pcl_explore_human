#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_explore_human/Time_Bool.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
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
      private_nh_.getParam("lrf_topic",lrf_topic_);
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

      sub_ = nh_.subscribe<sensor_msgs::PointCloud2> (lrf_topic_, 1, &ExtractHumansizeCloud::cloud_cb,this);
      pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("output_filter_cloud", 1);
      pub2_ = nh_.advertise<sensor_msgs::PointCloud2> ("output_humansize_cloud", 1);
      pub3_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("clustering_box",1);
      pub4_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("humansize_box",1);
      
      bool_sub_.subscribe(nh_,"area_flag",10);
      cloud_sub_.subscribe(nh_,"cloud",10);
      
      sync_.reset(new Synchronizer(SyncPolicy(10),bool_sub_,cloud_sub_));//,bool_sub_,cloud_sub_));
      sync_->registerCallback(boost::bind(&ExtractHumansizeCloud::mfcallback,this,_1,_2));

      tf_listener_ = new tf::TransformListener();

    }
  
  private:

    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void mfcallback(const pcl_explore_human::Time_BoolConstPtr& bool_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Publisher pub2_;
    ros::Publisher pub3_;
    ros::Publisher pub4_;

    message_filters::Subscriber<pcl_explore_human::Time_Bool> bool_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
    typedef message_filters::sync_policies::ApproximateTime<pcl_explore_human::Time_Bool, sensor_msgs::PointCloud2> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
    boost::shared_ptr<Synchronizer> sync_;

    tf::TransformListener *tf_listener_;

    std::string robot_frame_;
    std::string lrf_topic_;

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
    std::string path_ = ros::package::getPath("pcl_explore_human");
    std::string filename_;
    int cluster_number=1;   

};

void
ExtractHumansizeCloud::mfcallback(const pcl_explore_human::Time_BoolConstPtr& bool_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

}

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

  if(cloud->size()==0){
    return;
  }
  
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

   if(cloud_boxel->points.size()==0){
    return;
   }

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

  // Visualize clustering cloud
  jsk_recognition_msgs::BoundingBoxArray box_array;
  jsk_recognition_msgs::BoundingBox box;
  box.header.frame_id=robot_frame_;
  box_array.header.frame_id=robot_frame_;

  jsk_recognition_msgs::BoundingBoxArray box_array2;
  jsk_recognition_msgs::BoundingBox box2;
  box2.header.frame_id=robot_frame_;
  box_array2.header.frame_id=robot_frame_;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointXYZI min_pt, max_pt;

    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    {
      cloud_cluster->points.push_back (cloud_boxel->points[*pit]);
    }

    //Search min_point,max_point
    pcl::getMinMax3D(*cloud_cluster,min_pt,max_pt);

    // Estimate the XYZ centroid
    Eigen::Vector4f xyz_centroid;
    pcl::compute3DCentroid (*cloud_cluster, xyz_centroid);
    //std::cout<<"xyz_centroid"<<std::endl;
    //std::cout<<xyz_centroid[2]<<std::endl;
    box.pose.position.x=xyz_centroid[0];
    box.pose.position.y=xyz_centroid[1];
    box.pose.position.z=xyz_centroid[2];

    double target_size[3];
    // Calculation target size
		target_size[0] = fabs(max_pt.x - min_pt.x);
		target_size[1] = fabs(max_pt.y - min_pt.y);
		target_size[2] = fabs(max_pt.z - min_pt.z);

    box.dimensions.x=target_size[0];
    box.dimensions.y=target_size[1];
    box.dimensions.z=target_size[2];
    box_array.boxes.push_back(box);

    //ROS_INFO("%lf %lf %lf",target_size[0],target_size[1],target_size[2]);
    
    if(    ((target_size[0] < max_target_width_)  && (target_size[0] > min_target_width_))
		    && ((target_size[1] < max_target_depth_)  && (target_size[1] > min_target_depth_))
		    && ((target_size[2] < max_target_height_) && (target_size[2] > min_target_height_))
        && (xyz_centroid[2] <= 1.0)
        && (xyz_centroid[2] >= 0.1)
		)
		{
      //std::cout<<xyz_centroid[2]<<std::endl;
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      if(save_to_pcd_ == true){
        //save process
        pcl::PCDWriter writer;
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        std::stringstream ss;
        ss << path_<<"/dataset"<<"/pcd/"<< "cloud_cluster_" << cluster_number << ".pcd";
        ss >> filename_;
        std::cout <<"output_humansize_cloud save to "<< filename_ << std::endl;
        writer.write<pcl::PointXYZI> (ss.str (), *cloud_cluster, false); 
        cluster_number++;
      }

      if(cloud_cluster->points.size() > 1){
        sensor_msgs::PointCloud2 output_humansize_cloud;
				pcl::toROSMsg(*cloud_cluster, output_humansize_cloud);

				//Add header to output cloud
				output_humansize_cloud.header = cloud_msg->header;
				output_humansize_cloud.header.frame_id = robot_frame_;

				pub2_.publish(output_humansize_cloud);

        box2.pose.position.x=xyz_centroid[0];
        box2.pose.position.y=xyz_centroid[1];
        box2.pose.position.z=xyz_centroid[2];

        box2.dimensions.x=target_size[0];
        box2.dimensions.y=target_size[1];
        box2.dimensions.z=target_size[2];
        box_array2.boxes.push_back(box2);
        
        pub4_.publish(box_array2);
      }
    }
  }
  pub3_.publish(box_array);
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
