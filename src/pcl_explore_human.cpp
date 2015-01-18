
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

bool enforceIntensitySimilarity (const pcl::PointXYZINormal& point_a, const pcl::PointXYZINormal& point_b, float squared_distance)
{
  if (fabs (point_a.intensity - point_b.intensity) < 5.0f)
    return (true);
  else
    return (false);
}

bool enforceCurvatureOrIntensitySimilarity (const pcl::PointXYZINormal& point_a, const pcl::PointXYZINormal& point_b, float squared_distance)
{
  Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.normal, point_b_normal = point_b.normal;
  if (fabs (point_a.intensity - point_b.intensity) < 5.0f)
    return (true);
  if (fabs (point_a_normal.dot (point_b_normal)) < 0.05)
    return (true);
  return (false);
}

bool  customRegionGrowing (const pcl::PointXYZINormal& point_a, const pcl::PointXYZINormal& point_b, float squared_distance)
{
  Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.normal, point_b_normal = point_b.normal;
  if (squared_distance < 0.1)
  {
    if (fabs (point_a.intensity - point_b.intensity) < 8.0f)
      return (true);
    if (fabs (point_a_normal.dot (point_b_normal)) < 0.06)
      return (true);
  }
  else
  {
    if (fabs (point_a.intensity - point_b.intensity) < 3.0f)
      return (true);
  }
  return (false);
}


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

  //Eigen::Vector3f axis = Eigen::Vector3f(1.0,0.0,0.0);

  input->fields[3].name = "intensity";
  pcl::fromROSMsg(*input, *conv_input);

	pcl::VoxelGrid<pcl::PointXYZI> vg;
	vg.setInputCloud(conv_input);
  vg.setLeafSize(0.025,0.025,0.025);
  vg.setDownsampleAllData(true);
  vg.filter(*cloud_boxel);

  pcl::SACSegmentation<pcl::PointXYZI> seg;
  //seg.setEpsAngle( 180.0f * (M_PI/180.0f) );
  //seg.setAxis(axis);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.02);

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

  /*
  pcl::copyPointCloud (*cloud_boxel, *cloud_with_normals);
  pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> ne;
  ne.setInputCloud (cloud_boxel);
  ne.setSearchMethod (search_tree);
  ne.setRadiusSearch (0.15);
  ne.compute (*cloud_with_normals);

	pcl::ConditionalEuclideanClustering<pcl::PointXYZINormal> cec (true);
	cec.setInputCloud (cloud_with_normals);

  //cec.setConditionFunction (&enforceIntensitySimilarity);
  //cec.setConditionFunction (&enforceIntensitySimilarity);
  cec.setConditionFunction (&customRegionGrowing);
	// Points within this distance from one another are going to need to validate the enforceIntensitySimilarity function to be part of the same cluster:
	cec.setClusterTolerance (1.0f);
	// Size constraints for the clusters:
	cec.setMinClusterSize (20);
	cec.setMaxClusterSize (25000);
	// The resulting clusters (an array of pointindices):
	cec.segment (*clusters);
	// The clusters that are too small or too large in size can also be extracted separately:
	cec.getRemovedClusters (small_clusters, large_clusters);

	for (int i = 0; i < small_clusters->size (); ++i)
		for (int j = 0; j < (*small_clusters)[i].indices.size (); ++j)
			cloud_boxel->points[(*small_clusters)[i].indices[j]].intensity = -2.0;
	for (int i = 0; i < large_clusters->size (); ++i)
		for (int j = 0; j < (*large_clusters)[i].indices.size (); ++j)
			cloud_boxel->points[(*large_clusters)[i].indices[j]].intensity = +10.0;

	for (int i = 0; i < clusters->size (); ++i)
	{
		int label = rand () % 8;
		for (int j = 0; j < (*clusters)[i].indices.size (); ++j)
			cloud_boxel->points[(*clusters)[i].indices[j]].intensity = label;
	}
*/

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
  int size = cluster_indices.size();
  int color_max = 15000;
  for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();it != cluster_indices.end (); ++it)
  {
    int now_itr = it - cluster_indices.begin();
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
    {
      cloud_boxel -> points[*pit].intensity = ( 15000 / size ) * now_itr;
      //cloud_cluster->points.push_back (cloud_boxel->points[*pit]);
    }/*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;*/
  }


	//pcl::toROSMsg( *cloud_cluster, output );
  pcl::toROSMsg(*cloud_boxel, output);
  /*for(int k=0;k<4;k++){
    std::cout << output.header.frame_id<<std::endl;
    std::cout << input->header.frame_id<<std::endl;
  }*/


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
