#include<ros/ros.h>
#include<ros/package.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <std_msgs/Float32MultiArray.h>
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

#include <Eigen/Dense>

class ExtractHumanDescription{
    public:
        ExtractHumanDescription(){
            
            ros::NodeHandle private_nh_("~");
            private_nh_.getParam("output_screen",output_screen_);
            private_nh_.getParam("intensity_histgram_limit",intensity_histgram_limit_);
            private_nh_.getParam("intensity_histgram_bin",intensity_histgram_bin_);
            private_nh_.getParam("slice_sectors",slice_sectors_);
            private_nh_.getParam("is_save",is_save_);
            private_nh_.getParam("label",label_);
            private_nh_.getParam("description_filename",description_filename_);


            sub_=nh_.subscribe<sensor_msgs::PointCloud2>("output_humansize_cloud",1,&ExtractHumanDescription::cluster_cloud_cb,this);
            pub2_=nh_.advertise<std_msgs::Float32MultiArray>("description",1);
            pub3_=nh_.advertise<sensor_msgs::PointCloud2>("translate_cloud",1);

            tf_listener_ = new tf::TransformListener();

            ss<<path_<<"/dataset/"<<description_filename_;
            ss>>filename_;

        }
    private:
        void cluster_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        ros::Publisher pub2_;
        ros::Publisher pub3_;

         tf::TransformListener *tf_listener_;

        bool output_screen_;
        double intensity_histgram_limit_;
        int intensity_histgram_bin_;
        int slice_sectors_;
        bool is_save_;
        int label_;
        std::string description_filename_;
        std::stringstream ss;
        std::string filename_;
        std::string path_ = ros::package::getPath("pcl_explore_human");

};

void 
ExtractHumanDescription::cluster_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

    //Calculate robotpose from map_frame to robot_frame
    tf::StampedTransform transform;
    sensor_msgs::PointCloud2 transformed_cloud;

    try{
		tf_listener_->waitForTransform("odom",cloud_msg->header.frame_id, ros::Time(0), ros::Duration(100.0));
        tf_listener_->lookupTransform("odom", cloud_msg->header.frame_id, ros::Time(0), transform);
	}catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}

    std::vector<double>robot_pose(3);
    robot_pose[0] = transform.getOrigin().x();
    robot_pose[1] = transform.getOrigin().y();
    robot_pose[2] = transform.getOrigin().z();

    //std::cout<<"x="<<robot_pose[0]<<" "<<"y="<<robot_pose[1]<<" "<<"z="<<robot_pose[2]<<std::endl;
	
    //pcl to rosmsg
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*cloud_msg,*cloud);

    //Calculate cloud size
    int cloud_size = cloud->points.size();

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

    // Estimate the XYZ centroid
    Eigen::Vector4f xyz_centroid;
    pcl::compute3DCentroid (*cloud, xyz_centroid);

    //Calculate target_point from odom
    std::cout<<"target_point"<<std::endl;
    std::vector<double> target_point(3);
    for(int i=0;i<target_point.size();i++){
        target_point[i]=xyz_centroid[0]+robot_pose[i];
        std::cout<<target_point[i]<<std::endl;
    }
    std::cout<<std::endl;
    
    //PCA
    pcl::PCA<pcl::PointXYZI> pca;
    Eigen::Vector3f eigen_values;
	Eigen::Matrix3f eigen_vectors;

    pca.setInputCloud(cloud);

    eigen_values = pca.getEigenValues();
    //std::cout << "Eigen values of PCAed pointcloud: " << eigen_values[0] << ", "  << eigen_values[1] << ", " << eigen_values[2] << std::endl;
    eigen_vectors << pca.getEigenVectors();
    //std::cout << eigen_vectors <<std::endl;

    //Rotation cloud from PCA data
    //double theta=0.296706;  //17deg
    double theta;
	theta = atan2(eigen_vectors(1,0), eigen_vectors(0,0));
	//std::cout << "Theta: " << theta << std::endl;

    Eigen::Affine3f translate = Eigen::Affine3f::Identity();
	Eigen::Affine3f rotate = Eigen::Affine3f::Identity();
    pcl::PointCloud<pcl::PointXYZI>::Ptr transform_cloud_translate(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr transform_cloud_rotate(new pcl::PointCloud<pcl::PointXYZI>());

    translate.translation() << -xyz_centroid[0], -xyz_centroid[1], -xyz_centroid[2];
	pcl::transformPointCloud(*cloud, *transform_cloud_translate, translate);

    rotate.rotate(Eigen::AngleAxisf(-theta, Eigen::Vector3f::UnitZ()));
	pcl::transformPointCloud(*transform_cloud_translate, *transform_cloud_rotate, rotate);

    sensor_msgs::PointCloud2 translate_output;
    pcl::toROSMsg(*transform_cloud_rotate, translate_output);
	translate_output.header = cloud_msg -> header;
	translate_output.header.frame_id = "base_link";
    pub3_.publish(translate_output);


    Eigen::Vector4f transform_centroid;
    pcl::compute3DCentroid (*transform_cloud_rotate, transform_centroid);
    //std::cout << "transform_centroid: " << transform_centroid << std::endl;

    // Compute the 3x3 covariance matrix
    Eigen::Matrix3f covariance_matrix;
    pcl::computeCovarianceMatrixNormalized (*transform_cloud_rotate, transform_centroid, covariance_matrix);

    // moment of inertia buffer
    Eigen::Matrix3f moment_of_inertia_matrix_tmp = Eigen::Matrix3f::Zero();
	Eigen::Matrix3f moment_of_inertia_matrix = Eigen::Matrix3f::Zero();

    // intensity buffer
	double intensity_sum=0, intensity_ave=0, max_intensity=0, intensity_pow_sum=0, intensity_std_dev=0;
	std::vector<double> intensity_histgram(intensity_histgram_bin_);

    for(auto &&pit : *transform_cloud_rotate)
    {
        //Calculate three dimentional moment of inertia matrix
		moment_of_inertia_matrix_tmp << 
		powf(pit.y,2.0f)+powf(pit.z,2.0f),	-pit.x*pit.y,							-pit.x*pit.z,
		-pit.x*pit.y,						powf(pit.x,2.0f)+powf(pit.z,2.0f),	    -pit.y*pit.z,
		-pit.x*pit.z,						-pit.y*pit.z,						    powf(pit.x,2.0f)+powf(pit.y,2.0f);

		moment_of_inertia_matrix += moment_of_inertia_matrix_tmp/cloud_size;

        // Calculate intensity sum 
		intensity_sum += pit.intensity;
		intensity_pow_sum += powf(pit.intensity,2);
         // Calculate intensity distribution
		intensity_histgram[pit.intensity / (intensity_histgram_limit_ / intensity_histgram.size())] += 1;
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
        hist /= cloud_size;
    }

    //Calculate Slice distribution
    pcl::PointXYZI min_pt,max_pt,sliced_min_pt,sliced_max_pt;
    pcl::getMinMax3D(*transform_cloud_rotate, min_pt, max_pt);
	double sector_height = (max_pt.z - min_pt.z)/slice_sectors_;

    pcl::PassThrough<pcl::PointXYZI> pass;
	std::vector<std::vector<double> > slice_dist(slice_sectors_,std::vector<double> (2));

    for(double sec_h = min_pt.z,i = 0; sec_h < max_pt.z - sector_height; sec_h += sector_height, i++){
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_sliced(new pcl::PointCloud<pcl::PointXYZI>());
		pass.setInputCloud(transform_cloud_rotate);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(sec_h, sec_h + sector_height);
		pass.filter(*cloud_sliced);

        if(int(cloud_sliced->size())==0){
            slice_dist[i][0]=0;
            slice_dist[i][1]=0;
        }
        else{
		pcl::getMinMax3D(*cloud_sliced,sliced_min_pt,sliced_max_pt);

		slice_dist[i][0] = sliced_max_pt.x - sliced_min_pt.x;
		slice_dist[i][1] = sliced_max_pt.y - sliced_min_pt.y;
        }
	}

    if(output_screen_ == true){

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
		    for(int j=0;j<slice_sectors_;j++){
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

    //description container
    std_msgs::Float32MultiArray description;
    //f1
    description.data.push_back(cloud_size);
    //f2
    description.data.push_back(min_distance);
    //f3
    for(int i=0;i<int(covariance_matrix.size());i++){
		if(i<5 || i==6){
			description.data.push_back(covariance_matrix(i));
		}
    }
    //f4
    for(int i=0;i<int(moment_of_inertia_matrix.size());i++){
		if(i<5 || i==6){
			description.data.push_back(moment_of_inertia_matrix(i));
		}
    }
    //f5
    for(int i=0;i<2;i++){
		for(int j=0;j<10;j++){
			description.data.push_back(slice_dist[j][i]);
		}
    }   
    //f6
    description.data.push_back(intensity_ave);
	description.data.push_back(intensity_std_dev);
    for(int i=0; i<int(intensity_histgram.size()); i++){
		description.data.push_back(intensity_histgram[i]);
	}

    pub2_.publish(description);    

    if(is_save_){
		std::ofstream ofs;
		ofs.open(filename_, std::ios::ate | std::ios::app);
		//ofs << "1 ";
        ofs<<label_<<",";
		for(int i=0;i<description.data.size();++i){
			//ofs << i+1 << ":" << description.data[i] << " ";
            ofs <<description.data[i];
            if(i!=description.data.size()-1){
                ofs<<",";
            }
		}
		ofs << std::endl;
		ofs.close();
	}
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"pcl_extraction_human_description");

    ExtractHumanDescription ex;

    ros::spin();
}