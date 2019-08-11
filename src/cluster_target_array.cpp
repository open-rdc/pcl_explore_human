#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Eigen/Dense>

class ClusterTargetArray{
    public:
        ClusterTargetArray(){
            sub_ =nh_.subscribe<geometry_msgs::PoseArray>("target_array",1,&ClusterTargetArray::callback,this);
            pub_ =nh_.advertise<sensor_msgs::PointCloud2>("target_array_pointcloud",1);
            pub2_=nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("covariance_target_point",1);
        }

    private:
        void callback(geometry_msgs::PoseArray array_msg);

        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        ros::Publisher pub_;
        ros::Publisher pub2_;

};
 
void 
ClusterTargetArray::callback(const geometry_msgs::PoseArray array_msg)
{
    //std::cout<<array_msg.poses.size()<<std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    
    //convert PoseArray to PointCloud
    for(auto &&pit : array_msg.poses){
        //std::cout<<"pt.x"<<std::endl;
        //std::cout<<pit.position.x<<std::endl;
        pcl::PointXYZI pt;
        pt.x=pit.position.x;
        pt.y=pit.position.y;
        pt.z=pit.position.z;
        pt.intensity=1;
        //std::cout<<pt.x<<std::endl;
        cloud->push_back(pt);
    }
    //std::cout<<cloud->size()<<std::endl;

    //Create fileter container
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
    
    // build the filter
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.2);
    outrem.setMinNeighborsInRadius (5);
    // apply filter
    outrem.filter (*cloud_filtered);    

    std::cout<<cloud_filtered->size()<<std::endl;
    sensor_msgs::PointCloud2 target_array_cloud;
	pcl::toROSMsg(*cloud_filtered, target_array_cloud);

	//Add header to output cloud
	target_array_cloud.header = array_msg.header;

	pub_.publish(target_array_cloud);

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance (0.2); 
    ec.setMinClusterSize (10);
    ec.setMaxClusterSize (100);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);

        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
        }

        geometry_msgs::PoseWithCovarianceStamped point;

        // Estimate the XYZ centroid
        Eigen::Vector4f xyz_centroid;
        pcl::compute3DCentroid (*cloud_filtered, xyz_centroid);
        std::cout<<"xyz_centroid"<<std::endl;
        std::cout<<xyz_centroid<<std::endl;

        // Compute the 3x3 covariance matrix
        Eigen::Matrix3f covariance_matrix;
        pcl::computeCovarianceMatrixNormalized (*cloud_filtered, xyz_centroid, covariance_matrix);
        std::cout<<"covariance_matrix"<<std::endl;
        std::cout<<covariance_matrix<<std::endl;

        Eigen::MatrixXf sample2 = covariance_matrix;
        sample2.conservativeResizeLike(Eigen::MatrixXf::Zero(6,6));

        //sample.block<3,3>(0,0)=covariance_matrix.block<3,3>(0,0);
        std::cout<<sample2<<std::endl;

        for(int i=0;i<36;i++){
             point.pose.covariance[i]=sample2(i);
        }

        point.header = array_msg.header;
        point.pose.pose.position.x=xyz_centroid[0];
        point.pose.pose.position.y=xyz_centroid[1];
        point.pose.pose.position.z=xyz_centroid[2];

        pub2_.publish(point);

    }

}

int main(int argc, char **argv)
{   
    ros::init(argc,argv,"cluster_target_array");

    ClusterTargetArray cl;

    ros::spin();

     return 0;
}