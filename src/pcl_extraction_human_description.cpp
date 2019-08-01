#include<ros/ros.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/filters/passthrough.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl/features/normal_3d.h>

#include <vector>
#include <math.h>

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

};

void 
ExtractHumanDescription::cluster_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*cloud_msg,*cloud);

    //Calculate cloud size
    int cloud_size = cloud->points.size();

    // Estimate the XYZ centroid
    Eigen::Vector4f xyz_centroid;
    pcl::compute3DCentroid (*cloud, xyz_centroid);

    // Compute the 3x3 covariance matrix
    Eigen::Matrix3f covariance_matrix;
    pcl::computeCovarianceMatrixNormalized (*cloud, xyz_centroid, covariance_matrix);

    //brute force nearest neighbor search
    pcl::PointXYZI searchPoint;
	searchPoint.x = 0.0;
	searchPoint.y = 0.0;
	searchPoint.z = 0.0;

	float min_distance,min_distance_tmp;

    min_distance = sqrt(
		    powf( cloud->points[0].x - searchPoint.x, 2.0f ) +
		    powf( cloud->points[0].y - searchPoint.y, 2.0f ) +
		    powf( cloud->points[0].z - searchPoint.z, 2.0f )
	        );

	for(auto &&pit : *cloud)
    {
		min_distance_tmp = sqrt(
			powf( pit.x - searchPoint.x, 2.0f ) +
			powf( pit.y - searchPoint.y, 2.0f ) +
			powf( pit.z - searchPoint.z, 2.0f )
		);

		if( min_distance < min_distance_tmp){
			min_distance = min_distance_tmp;
		}
	}

    // moment of inertia buffer
    Eigen::Matrix3f moment_of_inertia_matrix_tmp = Eigen::Matrix3f::Zero();
	Eigen::Matrix3f moment_of_inertia_matrix = Eigen::Matrix3f::Zero();

    // intensity buffer
	double intensity_sum=0, intensity_ave=0, max_intensity=0, intensity_pow_sum=0, intensity_std_dev=0, intensity_histgram_limit = 3000;
	std::vector<double> intensity_histgram(25);

    for(auto &&pit : *cloud)
    {
        //Calculate three dimentional moment of inertia matrix
		moment_of_inertia_matrix_tmp << 
		powf(pit.y,2.0f)+powf(pit.z,2.0f),	-pit.x*pit.y,							-pit.x*pit.z,
		-pit.x*pit.y,						powf(pit.x,2.0f)+powf(pit.z,2.0f),	    -pit.y*pit.z,
		-pit.x*pit.z,						-pit.y*pit.z,						    powf(pit.x,2.0f)+powf(pit.y,2.0f);

		moment_of_inertia_matrix += moment_of_inertia_matrix_tmp;

        // Calculate intensity sum 
		intensity_sum += pit.intensity;
		intensity_pow_sum += powf(pit.intensity,2);
         // Calculate intensity distribution
		intensity_histgram[pit.intensity / (intensity_histgram_limit / intensity_histgram.size())] += 1;
		if( max_intensity < pit.intensity ){
			max_intensity = pit.intensity;
		}
    }

    //Calculate intensity average
    intensity_ave = intensity_sum / cloud_size;

    //Calculate intensity standard deviation
	intensity_std_dev = sqrt(fabs(intensity_pow_sum / cloud_size - powf(intensity_ave,2)));

    //Normalize intensity histgram
    for(auto &&hist : intensity_histgram){
        hist = hist / cloud_size;
    }

    //Calculate Slice distribution
    pcl::PointXYZI min_pt,max_pt,sliced_min_pt,sliced_max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    int sectors = 10;
	double sector_height = (max_pt.z - min_pt.z)/sectors;

    pcl::PassThrough<pcl::PointXYZI> pass;
	std::vector<std::vector<double> > slice_dist(sectors,std::vector<double> (2));

    for(double sec_h = min_pt.z,i = 0; sec_h < max_pt.z - sector_height; sec_h += sector_height, i++){
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_sliced(new pcl::PointCloud<pcl::PointXYZI>());
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(sec_h, sec_h + sector_height);
		pass.filter(*cloud_sliced);

		pcl::getMinMax3D(*cloud_sliced,sliced_min_pt,sliced_max_pt);

		slice_dist[i][0] = sliced_max_pt.x - sliced_min_pt.x;
		slice_dist[i][1] = sliced_max_pt.y - sliced_min_pt.y;
	}

	for(int i=0;i<2;i++){
		for(int j=0;j<sectors;j++){
			//description.push_back(slice_dist[j][i]);
		}
	}

    bool see_description_param =true;

    if(see_description_param == true){

        std::cout<<"***EXTRACT_DESCRIPTION_PARAMETER***\n"<<std::endl;

        //f1
        std::cout<<"Cloud Size:\n"<< cloud_size <<std::endl;
        //f2
        std::cout<<"Min Distance:\n"<< min_distance <<std::endl;
        //f3
        std::cout<<"Covariance Matrix:\n"<< covariance_matrix <<std::endl;
        //f4
        std::cout<<"Moment of Inertia:\n"<< moment_of_inertia_matrix <<std::endl;

        //f5
        std::cout <<"Slice Distribution:" << std::endl;
        for(int i=0;i<2;i++){
		    for(int j=0;j<sectors;j++){
			    std::cout << slice_dist[j][i] << " ";
		    }
        }
        std::cout << std::endl;

        //f6
        std::cout << "Intensity Average:\n" << intensity_ave << std::endl;
        std::cout << "Intensity Standard Deviation:\n" << intensity_std_dev <<std::endl;
        std::cout << "Intensity Histgram:"<<std::endl;
        for(auto &&hist : intensity_histgram){
            std::cout<<hist<<" ";
        }
		std::cout << std::endl;

    }

}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"pcl_extraction_human_description");

    ExtractHumanDescription ex;

    ros::spin();
}