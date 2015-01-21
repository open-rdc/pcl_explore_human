
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
#include <pcl/filters/statistical_outlier_removal.h>
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
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>());

  //Conversion PointCloud2 intensity field name to PointXYZI intensity field name.
  input->fields[3].name = "intensity";
  pcl::fromROSMsg(*input, *conv_input);

	pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
	sor.setInputCloud(conv_input);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_filtered);

	pcl::VoxelGrid<pcl::PointXYZI> vg;
	vg.setInputCloud(cloud_filtered);
  vg.setLeafSize(0.005,0.005,0.005);
  vg.setDownsampleAllData(true);
  vg.filter(*cloud_boxel);

  //Plane segmentation
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
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

  tree->setInputCloud( cloud_boxel );
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(0.05);
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(100000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_boxel);
  ec.extract(cluster_indices);

  int j=0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
  /*---for Debug Visualize(intensity coloring)---*/
  int size = cluster_indices.size();
  int color_max = 15000;
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
    //High intensity judge
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
    {
      if( cloud_boxel -> points[*pit].intensity > 15000){
        is_highIntensity = true;
      }
    }
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
    {
      now_point = pit - it->indices.begin();
      /*---for Debug intensity set---*/
      if( !is_highIntensity ){
        cloud_boxel -> points[*pit].intensity = ( 15000 / size ) * now_cluster;
      }
      /*-----------------------------*/
      /*---measure size of high intensity index---*/
      else{
        if( now_point == 0 ){
          min_point[0] = cloud_boxel -> points[*pit].x;
          min_point[1] = cloud_boxel -> points[*pit].y;
          min_point[2] = cloud_boxel -> points[*pit].z;
          max_point[0] = cloud_boxel -> points[*pit].x;
          max_point[1] = cloud_boxel -> points[*pit].y;
          max_point[2] = cloud_boxel -> points[*pit].z;
        }
        if( min_point[0] > cloud_boxel -> points[*pit].x ){
          min_point[0] = cloud_boxel -> points[*pit].x;
        }
        else if( max_point[0] < cloud_boxel -> points[*pit].x ){
          max_point[0] = cloud_boxel -> points[*pit].x;
        }
        if( min_point[1] > cloud_boxel -> points[*pit].x ){
          min_point[1] = cloud_boxel -> points[*pit].x;
        }
        else if( max_point[1] < cloud_boxel -> points[*pit].x ){
          max_point[1] = cloud_boxel -> points[*pit].x;
        }
        if( min_point[2] > cloud_boxel -> points[*pit].x ){
          min_point[2] = cloud_boxel -> points[*pit].x;
        }
        else if( max_point[2] < cloud_boxel -> points[*pit].x ){
          max_point[2] = cloud_boxel -> points[*pit].x;
        }
      }
    }
    if( is_highIntensity ){
      std::cout<<"HighIntensity"<<std::endl;
      float width,depth,height;
      width = fabs(max_point[0] - min_point[0]);
      depth = fabs(max_point[1] - min_point[1]);
      height = fabs(max_point[2] - min_point[2]);
      std::cout << "size:" << width << "," << depth << "," << height << "," << std::endl;
      if( (width < 1.0 && width > 0.5) && (depth < 2.5 && depth > 1.0) && (height < 2.3 && height > 1.3 ) ){
        std::cout << "Find Target" << std::endl;
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
