#include<ros/ros.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/filters/passthrough.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>

#include <vector>

class ExtractHumanDescription{
    public:
        ExtractHumanDescription(){
            
            ros::NodeHandle private_nh_("~");
            sub_=nh_.subscribe<sensor_msgs::PointCloud2>("output_humansize_cloud",1,&ExtractHumanDescription::cluster_cloud_cb,this);
            pub_=nh_.advertise<geometry_msgs::PointStamped>("target_point",1);

        }
    private:
        void cluster_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        ros::Publisher pub_;

        tf::TransformListener *tf_listener;
};

void 
ExtractHumanDescription::cluster_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg,*cloud);

    int cloud_size = cloud->points.size();
    std::cout << "cloud_size: " << cloud_size << std::endl;

    pcl::MomentOfInertiaEstimation <pcl::PointXYZI> feature_extractor;
    feature_extractor.setInputCloud (cloud);
    feature_extractor.compute ();

    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;
    pcl::PointXYZI min_point_AABB;
    pcl::PointXYZI max_point_AABB;
    pcl::PointXYZI min_point_OBB;
    pcl::PointXYZI max_point_OBB;
    pcl::PointXYZI position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

    feature_extractor.getMomentOfInertia (moment_of_inertia);
    feature_extractor.getEccentricity (eccentricity);
    feature_extractor.getAABB (min_point_AABB, max_point_AABB);
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenValues (major_value, middle_value, minor_value);
    feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter (mass_center);

    // Placeholder for the 3x3 covariance matrix at each surface patch
    Eigen::Matrix3f covariance_matrix;
    // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
    Eigen::Vector4f xyz_centroid;

    // Estimate the XYZ centroid
    pcl::compute3DCentroid (*cloud, xyz_centroid);
    std::cout<<"xyz_centroid\n"<<xyz_centroid<<std::endl;

    // Compute the 3x3 covariance matrix
    pcl::computeCovarianceMatrixNormalized (*cloud, xyz_centroid, covariance_matrix);
    std::cout<<"CovarianceMatrixNormalized\n"<<covariance_matrix<<std::endl;

}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"pcl_extraction_human_description");

    ExtractHumanDescription ex;

    ros::spin();
}