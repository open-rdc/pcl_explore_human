
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
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
#include <pcl/features/normal_3d.h>
#include <pcl/PCLPointCloud2.h>

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2Ptr& input)
{
	// Create a container for the data.
	sensor_msgs::PointCloud2 output;
	pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters), small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::search::KdTree<pcl::PointXYZI>::Ptr search_tree (new pcl::search::KdTree<pcl::PointXYZI>);
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr conv_input(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_boxel(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>());

  //Conversion PointCloud2 intensity field name to PointXYZI intensity field name.
  input->fields[3].name = "intensity";
  pcl::fromROSMsg(*input, *conv_input);

	pcl::VoxelGrid<pcl::PointXYZI> vg;
	vg.setInputCloud(conv_input);
  vg.setLeafSize(0.025,0.025,0.025);
  vg.setDownsampleAllData(true);
  vg.filter(*cloud_boxel);

  //Plane segmentation
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  //Iteration count
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.02);

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

  tree->setInputCloud( cloud_boxel );
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(0.1);
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_boxel);
  ec.extract(cluster_indices);

  int j=0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
  /*---for Debug Visualize(intensity coloring)---*/
  //int size = cluster_indices.size();
  //int color_max = 15000;
  /*---------------------------------------------*/
  int now_cluster;
  for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();it != cluster_indices.end (); ++it)
  {
    //Now iterator count
    now_cluster = it - cluster_indices.begin();
    float min_point[3];
    float max_point[3];
    int now_point;
    bool is_highIntensity = false;
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
    {
      now_point = pit - it->indices.begin();
      if( now_point == 0 ){
        min_point[0] = cloud_boxel -> points[*pit].x;
        min_point[1] = cloud_boxel -> points[*pit].y;
        min_point[2] = cloud_boxel -> points[*pit].z;
        max_point[0] = cloud_boxel -> points[*pit].x;
        max_point[1] = cloud_boxel -> points[*pit].y;
        max_point[2] = cloud_boxel -> points[*pit].z;
      }
      if( (min_point[0] * min_point[1] * min_point[2]) > ( cloud_boxel -> points[*pit].x * cloud_boxel -> points[*pit].y * cloud_boxel -> points[*pit].z) ){
        min_point[0] = cloud_boxel -> points[*pit].x;
        min_point[1] = cloud_boxel -> points[*pit].y;
        min_point[2] = cloud_boxel -> points[*pit].z;
      }
      if( (max_point[0] * max_point[1] * max_point[2]) < ( cloud_boxel -> points[*pit].x * cloud_boxel -> points[*pit].y * cloud_boxel -> points[*pit].z) ){
        max_point[0] = cloud_boxel -> points[*pit].x;
        max_point[1] = cloud_boxel -> points[*pit].y;
        max_point[2] = cloud_boxel -> points[*pit].z;
      }
      if( cloud_boxel -> points[*pit].intensity > 10000){
          is_highIntensity = true;
      }
      /*---for Debug intensity set---*/
      //cloud_boxel -> points[*pit].intensity = ( 15000 / size ) * now_cluster;
      /*-----------------------------*/
    }
    if( is_highIntensity ){
      std::cout<<"HighIntensity"<<std::endl;
      float width,depth,height;
      width = max_point[0] - min_point[0];
      depth = max_point[1] - min_point[1];
      height = max_point[2] - min_point[2];
      std::cout << width << "," << depth << "," << height << "," << std::endl;
      if( (width < 0.9 && width > 0.5) && (depth < 0.9 && depth > 0.5) && (height < 1.6 && height > 1.0 ) ){
        std::cout << width / 2 << "," << depth/2 << "," << height/2 << "," << std::endl;
      }
    }
  }

  pcl::toROSMsg(*cloud_boxel, output);


	// Publish the data.
	pub.publish (output);
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_node");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
