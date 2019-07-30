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

class ExtractHumanDescription{
    public:
        ExtractHumanDescription(){
            
            ros::NodeHandle private_nh_("~");
            sub_=nh_.subscribe<sensor_msgs::PointCloud2>("output_humansize_cloud",1,&ExtractHumanDescription::cluster_cloud_cb,this);

        }
    private:
        void cluster_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        ros::Publisher pub_;

        tf::TransformListener *tf_listener;
};

int main(int argc, char** argv)
{
    ros::init(argc,argv,"pcl_extraction_human_description");

    ros::spin();
}