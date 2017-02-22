// +ROS specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
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
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/PCLPointCloud2.h>

#include <stdio.h>
#include <sys/time.h>

ros::Publisher pub;
tf::TransformListener *tf_listener;

int a = 0;
int b = 0;
int l = 0;
int c = 0;
int z = 0;

int cut_count = 0;


void cloud_cb (const sensor_msgs::PointCloud2Ptr& input)
{
	// Create a container for the data.
	sensor_msgs::PointCloud2 transformed_cloud;
	sensor_msgs::PointCloud2 output;
	pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr conv_input(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_boxel(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_boxel_ground(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_outlier_filtered(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_boxel_count(new pcl::PointCloud<pcl::PointXYZI>());



	/*--- for debug ---*/
	bool sacSegmentFlag = false; 
	bool is_save = false;
	bool highIntensity_point = false;
	bool is_output = false;

	struct timeval s, f;
	//Get start time
	gettimeofday(&s, NULL);
	/*---           ---*/
	
	// Transform pointcloud from LIDAR fixed link to base_link
	std::string target_link_name = "/base_link";
	tf::StampedTransform transform;
	try{
		tf_listener->waitForTransform(target_link_name, input->header.frame_id, ros::Time(), ros::Duration(100.0));
	}catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	
	if(!pcl_ros::transformPointCloud(target_link_name, *input, 	transformed_cloud, *tf_listener)){
		return;
	}
	

	//Conversion PointCloud2 intensity field name to PointXYZI intensity field name.
	transformed_cloud.fields[3].name = "intensity";
	pcl::fromROSMsg(transformed_cloud, *conv_input);


	//Judge reflection intensity
	std::vector<pcl::PointIndices> cluster_indices_one;
	pcl::EuclideanClusterExtraction<pcl::PointXYZI> ee;
	ee.setSearchMethod(tree);
	ee.setInputCloud(conv_input);
	ee.extract(cluster_indices_one);

	int cluster_one;
	int i=0;
	int j=0;
	float intensity_value[20000][4];
	int HighIntensity_count = 0;
	bool loop = true;
	int reflection_intensity = 2000; //2000 - 1200


	while(loop){
		for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_one.begin();it != cluster_indices_one.end (); ++it)
		{
			pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_all_filtered(new pcl::PointCloud<pcl::PointXYZI>());
			
			cluster_one = it - cluster_indices_one.begin();
			int now_point;
			
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			{
				if( (conv_input -> points[*pit].intensity > reflection_intensity) && (conv_input -> points[*pit].x < 7.5) && (-7.5 < conv_input -> points[*pit].y < 7.5) ){
	
					i++; // i=1
					cut_count = 1;
					HighIntensity_count++;
					highIntensity_point = true;
					intensity_value[i][0]= conv_input -> points[*pit].x; 
					intensity_value[i][1]= conv_input -> points[*pit].y;
					intensity_value[i][2]= conv_input -> points[*pit].z;
					intensity_value[i][3]= conv_input -> points[*pit].intensity;

					break;
				}
			}
		}
		if((!highIntensity_point) && (reflection_intensity > 1200)){
			reflection_intensity = reflection_intensity - 200;
			loop = true;
		}else{
			loop = false;
		}
	}
	std::cout<<"Refrection intensity : "<< reflection_intensity <<std::endl;



//sort

/*	int g, h;
	float temp;
	for(g = 1; g <= i; g++){
		for(h = i; h > g; h--){
			if(intensity_value[h-1][0] > intensity_value[h][0]){
				temp = intensity_value[h][0];
				intensity_value[h][0] = intensity_value[h-1][0];
				intensity_value[h-1][0] = temp;
			}
		}
	}
	for(j = 1; j <= i; j++){
		std::cout<<"XYZ save_sort[x] : ["<<j<<"]"<<"X: "<<intensity_value[j][0]<<" , Y: "<<intensity_value[j][1]<<" ,  Z: "<<intensity_value[j][2]<<std::endl;
	}
*/

	if(i == 0){
		i = 1;
		intensity_value[i][0]= 0; 
		intensity_value[i][1]= 0;
		intensity_value[i][2]= 0;
		intensity_value[i][3]= 0;
	}

	float max_point[2][2] = {0}; //MAX: [0]:X, [1]:Y, [2]:Z , IN: [0]:X, [1] :Y, [2]:Z   
	float min_point[2][2] = {0}; 

	max_point[0][0] = intensity_value[1][0]; 
	max_point[0][1] = intensity_value[1][1]; 
	max_point[0][2] = intensity_value[1][2]; 
	min_point[0][0] = intensity_value[1][0]; 
	min_point[0][1] = intensity_value[1][1]; 
	min_point[0][2] = intensity_value[1][2]; 


	max_point[1][0] = intensity_value[1][0]; 
	max_point[1][1] = intensity_value[1][1]; 
	max_point[1][2] = intensity_value[1][2]; 
	min_point[1][0] = intensity_value[1][0]; 
	min_point[1][1] = intensity_value[1][1]; 
	min_point[1][2] = intensity_value[1][2]; 


	max_point[2][0] = intensity_value[1][0]; 
	max_point[2][1] = intensity_value[1][1]; 
	max_point[2][2] = intensity_value[1][2]; 
	min_point[2][0] = intensity_value[1][0]; 
	min_point[2][1] = intensity_value[1][1]; 
	min_point[2][2] = intensity_value[1][2]; 


	for(j = 1; j <= i; j++)
	{
		std::cout<<"XYZ save : ["<<j<<"]"<<"X: "<<intensity_value[j][0]<<" , Y: "<<intensity_value[j][1]<<" ,  Z: "<<intensity_value[j][2]<<" ,  I: "<<intensity_value[j][3]<<std::endl;
		
		if( min_point[0][0] > intensity_value[j][0] ){
			min_point[0][0] = intensity_value[j][0];
			min_point[0][1] = intensity_value[j][1];
			min_point[0][2] = intensity_value[j][2];
		}
		else if( max_point[0][0] < intensity_value[j][0] ){
			max_point[0][0] = intensity_value[j][0];
			max_point[0][1] = intensity_value[j][1];
			max_point[0][2] = intensity_value[j][2];
		}
		if( min_point[1][1] > intensity_value[j][1] ){
			min_point[1][0] = intensity_value[j][0];
			min_point[1][1] = intensity_value[j][1];
			min_point[1][2] = intensity_value[j][2];
		}
		else if( max_point[1][1] < intensity_value[j][1] ){
			max_point[1][0] = intensity_value[j][0];
			max_point[1][1] = intensity_value[j][1];
			max_point[1][2] = intensity_value[j][2];
		}
		if( min_point[2][2] > intensity_value[j][2] ){
			min_point[2][0] = intensity_value[j][0];
			min_point[2][1] = intensity_value[j][1];
			min_point[2][2] = intensity_value[j][2];
		}
		else if( max_point[2][2] < intensity_value[j][2] ){
			max_point[2][0] = intensity_value[j][0]; 
			max_point[2][1] = intensity_value[j][1]; 
			max_point[2][2] = intensity_value[j][2];  
		}
	}

	std::cout<<"MAX POINT [X] :" << "x : "<<max_point[0][0] <<"  y : "<<max_point[0][1]<<"  z : "<<max_point[0][2]<<std::endl;
	std::cout<<"MAX POINT [Y] :" << max_point[1][1] <<std::endl;
	std::cout<<"MIN POINT [Y] :" << min_point[1][1] <<std::endl;
	std::cout<<"MAX POINT [Z] :" << max_point[2][2] <<std::endl;


	pcl::VoxelGrid<pcl::PointXYZI> vg; 
	vg.setInputCloud(conv_input);
	vg.setLeafSize(0.15,0.15,0.15);
	vg.setDownsampleAllData(true);
	vg.filter(*cloud_boxel);

	pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
	sor.setInputCloud(cloud_boxel);
	sor.setMeanK(10);
	sor.setStddevMulThresh(0.25);
	sor.filter(*cloud_boxel);


	if(!sacSegmentFlag){
		/*
		int f;
		double e = 0;
		int save_number[10000];
		for(f = 0; f < 20; f++){
			pcl::PassThrough<pcl::PointXYZI> ground;
			ground.setFilterLimitsNegative (true);
			ground.setInputCloud(cloud_boxel);
			ground.setFilterFieldName("z");
			ground.setFilterLimits(-50.0, min_point[2]-0.05 - e/20);
			ground.filter(*cloud_boxel_count);
			pcl::PassThrough<pcl::PointXYZI> back;
			back.setFilterLimitsNegative (true);
			back.setInputCloud(cloud_boxel_count);
			back.setFilterFieldName("z");
			back.setFilterLimits(min_point[2] - e/20, 50.0);
			back.filter(*cloud_boxel_count);
	
			e++;
	
			std::vector<pcl::PointIndices> cluster_indices_one;
			pcl::EuclideanClusterExtraction<pcl::PointXYZI> en;
			en.setSearchMethod(tree);
			en.setInputCloud(cloud_boxel_count);
			en.extract(cluster_indices_one);
	
			int cluster_one;
			//float intensity_point[1000];  
			//pcl::PointXYZI intensity_pt; 
			int i=0;
			int j=0;
			float intensity_value[10000][2];
		
			for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_one.begin();it != cluster_indices_one.end (); ++it)
			{
				pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_all_filtered(new pcl::PointCloud<pcl::PointXYZI>());
				
				cluster_one = it - cluster_indices_one.begin();
				int now_point;
				
				for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
				{
					if( conv_input -> points[*pit].intensity > 0){
		
							//std::cout<<"count_wall [" <<d<<"]"<<" this wall number :  "<<i<<std::endl;
		
						i++; // i=1
		
						intensity_value[i][0]= conv_input -> points[*pit].x; 
						intensity_value[i][1]= conv_input -> points[*pit].y;
						intensity_value[i][2]= conv_input -> points[*pit].z;
		
						//std::cout<<"XYZ test : "<<intensity_value[i][0]<<" , "<<intensity_value[i][1]<<" , "<<intensity_value[i][2]<<std::endl;
						break;
					}
				}
			}
			save_number[f] = i;
			std::cout<<"count_ground [" <<f<<"]"<<" SUM this wall number :  "<< save_number[f] <<std::endl;
		}
		for(f = 0; f < 20; f++){
			if((save_number[f] > 100) && (save_number[f] = save_number[f-1] * 10)){
				pcl::PassThrough<pcl::PointXYZI> pass;
				pass.setFilterLimitsNegative (true);
				pass.setInputCloud(cloud_boxel);
				pass.setFilterFieldName("z");
				pass.setFilterLimits(-2.0, min_point[2] - f * 0.05);
				pass.filter(*cloud_boxel);
				std::cout<<"cut"<<std::endl;
				break;
			}else{
				pcl::PassThrough<pcl::PointXYZI> pass;
				pass.setFilterLimitsNegative (true);
				pass.setInputCloud(cloud_boxel);
				pass.setFilterFieldName("z");
				pass.setFilterLimits(-2.0,0.1);
				pass.filter(*cloud_boxel);
			}	
		}	
		*/

		pcl::PassThrough<pcl::PointXYZI> pass;
		pass.setFilterLimitsNegative (true);
		pass.setInputCloud(cloud_boxel);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(-2.0,0.03);
		pass.filter(*cloud_boxel_ground);
		
	}else{
		//Plane segmentation
		pcl::SACSegmentation<pcl::PointXYZI> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setAxis(Eigen::Vector3f(0,0,1));
		seg.setEpsAngle (20);
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
	}

//wall point couont
/*	int d;
	double e = 0;
	int save_number[10000];
	if(highIntensity_point){
		for(d = 0; d < 20; d++){
			pcl::PassThrough<pcl::PointXYZI> right_test;
			right_test.setFilterLimitsNegative (true);
			right_test.setInputCloud(cloud_boxel);
			right_test.setFilterFieldName("y");
			right_test.setFilterLimits(-50.0 , max_point[0][1]-0.5);
			right_test.filter(*cloud_boxel_count);
	
			pcl::PassThrough<pcl::PointXYZI> left_test;
			left_test.setFilterLimitsNegative (true);
			left_test.setInputCloud(cloud_boxel_count);
			left_test.setFilterFieldName("y");
			left_test.setFilterLimits(max_point[0][1]+0.5 , 50.0);
			left_test.filter(*cloud_boxel_count);
	
			pcl::PassThrough<pcl::PointXYZI> above_test;
			above_test.setFilterLimitsNegative (true);
			above_test.setInputCloud(cloud_boxel_count);
			above_test.setFilterFieldName("z");
			above_test.setFilterLimits(max_point[0][2]+1.0,50.0);
			above_test.filter(*cloud_boxel_count);

			pcl::PassThrough<pcl::PointXYZI> below;
			below.setFilterLimitsNegative (true);
			below.setInputCloud(cloud_boxel_count);
			below.setFilterFieldName("z");
			below.setFilterLimits(-2.0, max_point[0][2]-0.5);
			below.filter(*cloud_boxel_count);
	
			pcl::PassThrough<pcl::PointXYZI> front;
			front.setFilterLimitsNegative (true);
			front.setInputCloud(cloud_boxel_count);
			front.setFilterFieldName("x");
			front.setFilterLimits(-50.0, max_point[0][0]+e/20);
			front.filter(*cloud_boxel_count);

			pcl::PassThrough<pcl::PointXYZI> back;
			back.setFilterLimitsNegative (true);
			back.setInputCloud(cloud_boxel_count);
			back.setFilterFieldName("x");
			back.setFilterLimits(max_point[0][0]+e/20+0.05 , 50.0);
			back.filter(*cloud_boxel_count);
			e++;

			std::vector<pcl::PointIndices> cluster_indices_one;
			pcl::EuclideanClusterExtraction<pcl::PointXYZI> en;
			en.setSearchMethod(tree);
			en.setInputCloud(cloud_boxel_count);
			en.extract(cluster_indices_one);
			int cluster_one;
			//float intensity_point[1000];  
			//pcl::PointXYZI intensity_pt; 
			int i=0;
			int j=0;
			float intensity_value[10000][2];
			int save_number[10000];
		
			for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_one.begin();it != cluster_indices_one.end (); ++it)
			{
				pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_all_filtered(new pcl::PointCloud<pcl::PointXYZI>());
				
				cluster_one = it - cluster_indices_one.begin();
				int now_point;
				
				for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
				{
					if( conv_input -> points[*pit].intensity > 0){
		
						//std::cout<<"count_wall [" <<d<<"]"<<" this wall number :  "<<i<<std::endl;
		
						i++; // i=1
		
						intensity_value[i][0]= conv_input -> points[*pit].x; 
						intensity_value[i][1]= conv_input -> points[*pit].y;
						intensity_value[i][2]= conv_input -> points[*pit].z;
		
						//std::cout<<"XYZ test : "<<intensity_value[i][0]<<" , "<<intensity_value[i][1]<<" , "<<intensity_value[i][2]<<std::endl;
						break;
					}
				}
			}
			save_number[d] = i;
			std::cout<<"count_ground [" <<d<<"]"<<" SUM this wall number :  "<< save_number[d] <<std::endl;
		}
	}else{
		std::cout<<"Thie cloud has not highIntensity points so not wall exception"<<std::endl;
	}
	
*/	
	//ROS_INFO_STREAM( "is SACSegmentation" << sacSegmentFlag );

	

	tree->setInputCloud( cloud_boxel_ground );

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
	ec.setClusterTolerance(0.2);
	ec.setMinClusterSize(10);
	ec.setMaxClusterSize(1000000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_boxel_ground);
	ec.extract(cluster_indices);


	//int j=0;
	/*---for Debug Visualize(intensity coloring)---*/
	
	int size = cluster_indices.size();

	//int color_max = 15000;
	/*---------------------------------------------*/
	int now_cluster;
	int name_count = 0;

	ROS_INFO_STREAM("Number of clusters : " << cluster_indices.size());

	for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_all_filtered(new pcl::PointCloud<pcl::PointXYZI>());
		//Now iterator count
		now_cluster = it - cluster_indices.begin();
		float min_point_local[3];
		float max_point_local[3];
		pcl::PointXYZI min_pt,max_pt;
		int now_point;

		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
		{
			cloud_all_filtered->points.push_back(cloud_boxel_ground->points[*pit]);
			now_point = pit - it->indices.begin();
			/*---measure size of high intensity index---*/
			if( now_point == 0 ){
				min_pt = cloud_boxel_ground -> points[*pit];
				max_pt = cloud_boxel_ground -> points[*pit];
			}
			if( min_pt.x > cloud_boxel_ground -> points[*pit].x ){
				min_pt.x = cloud_boxel_ground -> points[*pit].x;
			}
			else if( max_pt.x < cloud_boxel_ground -> points[*pit].x ){ 
				max_pt.x = cloud_boxel_ground -> points[*pit].x;
			}
			if( min_pt.y > cloud_boxel_ground -> points[*pit].y ){
				min_pt.y = cloud_boxel_ground -> points[*pit].y;
			}
			else if( max_pt.y < cloud_boxel_ground -> points[*pit].y ){
				max_pt.y = cloud_boxel_ground -> points[*pit].y;
			}
			if( min_pt.z > cloud_boxel_ground -> points[*pit].z ){
				min_pt.z = cloud_boxel_ground -> points[*pit].z;
			}
			else if( max_pt.z < cloud_boxel_ground -> points[*pit].z ){
				max_pt.z = cloud_boxel_ground -> points[*pit].z;
			}
		}

		//std::cout <<  "maxmin XYZ : " <<"max.X: "<< max_pt.x << ", min.X: " << min_pt.x <<", max.Y: "<< max_pt.y << ", min.Y: " << min_pt.y <<", max.Z: "<< max_pt.z << ", min.Z: " << min_pt.z << std::endl;

		int d = 0;  
		int k = 1;

 		//Judge high reflection intensity in this cluster
		while(d == 0){
			if((k <= i) && (i != 0)){
			if( (max_pt.x > intensity_value[k][0]) && (intensity_value[k][0] > min_pt.x) && (max_pt.y > intensity_value[k][1]) && ( intensity_value[k][1] > min_pt.y) && (max_pt.z > intensity_value[k][2]) && (intensity_value[k][2] > min_pt.z) )
			{
				//if( (max_pt.x >= intensity_value[k][0]) && (intensity_value[k][0] >= min_pt.x) && (max_pt.y >= intensity_value[k][1]) && ( intensity_value[k][1] >= min_pt.y) && (max_pt.z >= intensity_value[k][2]) && (intensity_value[k][2] >= min_pt.z) ){
					d = 1;
				}else{
					k++;
				}
			}else{
				d = 2;
 			}
		}

		if(d == 1){
			printf("This cluster includes high points reflection intensity");
			printf("\n");
			}

			int file_cnt = 0;
			float width,depth,height;
			width = fabs(max_pt.x - min_pt.x);
			depth = fabs(max_pt.y - min_pt.y);
			height = fabs(max_pt.z - min_pt.z);

			//std::cout << "size:" << width << "," << depth << "," << height << "," << std::endl;

			if( ((width < 1.2) && (width > 0.4)) && ((depth < 1.2) && (depth > 0.4)) && ((height < 2.0) && (height > 1.0)) ){
			//if( ((width < 1.0) && (width > 0.5)) && ((depth < 1.0) && (depth > 0.5)) && ((height < 1.8) && (height > 1.0)) ){
				std::cout << "This cluster is a size of the target" << std::endl;

				if(d == 1){
					std::cout << "Find Target" << std::endl;
					std::cout <<  "Target Size : " <<"width: "<< width << ", depth: " << depth << ", height: " << height << std::endl;
					
					//Save process
					cloud_all_filtered -> width = 1;
					cloud_all_filtered -> height = cloud_all_filtered -> points.size();

					if(is_save){
						std::string file_path = "/home/amano/1108/";
						name_count++;
						std::stringstream file_name_st;
						std::string file_name;
						if(reflection_intensity == 2000){
							file_name_st << input->header.stamp << name_count <<"_first";
						}else if(reflection_intensity == 1800){
							file_name_st << input->header.stamp << name_count <<"_second";
						}else if(reflection_intensity == 1600){
							file_name_st << input->header.stamp << name_count <<"_third";
						}else if(reflection_intensity == 1400){
							file_name_st << input->header.stamp << name_count <<"_forth";
						}else{
							file_name_st << input->header.stamp << name_count <<"_fifth";
						}
						file_name.append(file_path);
						file_name.append(file_name_st.str());
						file_name.append(".pcd");
						std::cout << file_name <<std::endl;
						pcl::io::savePCDFileASCII(file_name, *cloud_all_filtered);
						file_cnt++;
						a++;
					}
				}else{   
					//if cloud is not target, leaving only one point to erase all.
					cloud_all_filtered->erase(cloud_all_filtered->begin()+1,cloud_all_filtered->end());
				}
			}else{
				//if cloud is not target, leaving only one point to erase all.
				cloud_all_filtered->erase(cloud_all_filtered->begin()+1,cloud_all_filtered->end());
			}

		if( cloud_all_filtered->points.size() > 1 ){
			pcl::toROSMsg(*cloud_all_filtered, output);
			// pcl::toROSMsg(*cloud_boxel, output);

			//add header to output cloud.
			output.header = input -> header;
			output.header.frame_id = "base_link";

			// Publish the data.
			pub.publish (output);
			is_output = true;
			b++;
		}
	}

	if((HighIntensity_count > 4) && (!is_output) ){
		pcl::PassThrough<pcl::PointXYZI> back;
		back.setFilterLimitsNegative (true);
		back.setInputCloud(cloud_boxel);
		back.setFilterFieldName("x");
		back.setFilterLimits(max_point[0][0]+0.3 , 50.0);
		back.filter(*cloud_boxel);

		pcl::PassThrough<pcl::PointXYZI> above;
		above.setFilterLimitsNegative (true);
		above.setInputCloud(cloud_boxel);
		above.setFilterFieldName("z");
		above.setFilterLimits(max_point[2][2]+1.0,50.0);
		above.filter(*cloud_boxel);	

		pcl::PassThrough<pcl::PointXYZI> below;
		below.setFilterLimitsNegative (true);
		below.setInputCloud(cloud_boxel);
		below.setFilterFieldName("z");
		below.setFilterLimits(-2.0,0.03);
		below.filter(*cloud_boxel);

		pcl::PassThrough<pcl::PointXYZI> right;
		right.setFilterLimitsNegative (true);
		right.setInputCloud(cloud_boxel);
		right.setFilterFieldName("y");
		right.setFilterLimits(-50.0 , min_point[1][1]-0.5);
		right.filter(*cloud_boxel);

		pcl::PassThrough<pcl::PointXYZI> left;
		left.setFilterLimitsNegative (true);
		left.setInputCloud(cloud_boxel);
		left.setFilterFieldName("y");
		left.setFilterLimits(max_point[1][1]+0.5 , 50.0);
		left.filter(*cloud_boxel);

		tree->setInputCloud( cloud_boxel );

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZI> eb;
		eb.setClusterTolerance(0.2);
		eb.setMinClusterSize(10);
		eb.setMaxClusterSize(1000000);
		eb.setSearchMethod(tree);
		eb.setInputCloud(cloud_boxel);
		eb.extract(cluster_indices);


		int size = cluster_indices.size();

		int now_cluster;
		int name_count = 0;

		ROS_INFO_STREAM("Number of clusters : " << cluster_indices.size());

		for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();it != cluster_indices.end (); ++it)
		{
			pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_all_filtered(new pcl::PointCloud<pcl::PointXYZI>());
			//Now iterator count
			now_cluster = it - cluster_indices.begin();
			float min_point_local[3];
			float max_point_local[3];
			pcl::PointXYZI min_pt,max_pt;
			int now_point;
	
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			{
				cloud_all_filtered->points.push_back(cloud_boxel->points[*pit]);
				now_point = pit - it->indices.begin();
				//---measure size of high intensity index---
				if( now_point == 0 ){
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
			}

			//std::cout <<  "maxmin XYZ : " <<"max.X: "<< max_pt.x << ", min.X: " << min_pt.x <<", max.Y: "<< max_pt.y << ", min.Y: " << min_pt.y <<", max.Z: "<< max_pt.z << ", min.Z: " << min_pt.z << std::endl;

		int d = 0;        
		int k = 1;

 		 //Judge high reflection intensity in this cluster
		while(d == 0){
			if((k <= i) && (i != 0)){
			if( (max_pt.x > intensity_value[k][0]) && (intensity_value[k][0] > min_pt.x) && (max_pt.y > intensity_value[k][1]) && ( intensity_value[k][1] > min_pt.y) && (max_pt.z > intensity_value[k][2]) && (intensity_value[k][2] > min_pt.z) )
			{
				//if( (max_pt.x >= intensity_value[k][0]) && (intensity_value[k][0] >= min_pt.x) && (max_pt.y >= intensity_value[k][1]) && ( intensity_value[k][1] >= min_pt.y) && (max_pt.z >= intensity_value[k][2]) && (intensity_value[k][2] >= min_pt.z) ){
					d = 1;
				}else{
					k++;
				}
			}else{
				d = 2;
 			}
		}

		if(d == 1){
			printf("This cluster includes high points reflection intensity");
			printf("\n");
			}

			int file_cnt = 0;
			float width,depth,height;
			width = fabs(max_pt.x - min_pt.x);
			depth = fabs(max_pt.y - min_pt.y);
			height = fabs(max_pt.z - min_pt.z);

			//std::cout << "size:" << width << "," << depth << "," << height << "," << std::endl;

			if( ((width < 1.2) && (width > 0.4)) && ((depth < 1.2) && (depth > 0.4)) && ((height < 2.0) && (height > 1.0)) ){
			//if( ((width < 1.0) && (width > 0.5)) && ((depth < 1.0) && (depth > 0.5)) && ((height < 1.8) && (height > 1.0)) ){
				std::cout << "This cluster is a size of the target" << std::endl;

				if(d == 1){
					std::cout << "Find Target" << std::endl;
					std::cout <<  "Target Size : " <<"width: "<< width << ", depth: " << depth << ", height: " << height << std::endl;

					//Save process
					cloud_all_filtered -> width = 1;
					cloud_all_filtered -> height = cloud_all_filtered -> points.size();

					if(is_save){
						std::string file_path = "/home/amano/1108/";
						name_count++;
						std::stringstream file_name_st;
						std::string file_name;
						file_name_st << input->header.stamp << name_count << "_cut_" << cut_count;
						file_name.append(file_path);
						file_name.append(file_name_st.str());
						file_name.append(".pcd");
						std::cout << file_name <<std::endl;
						pcl::io::savePCDFileASCII(file_name, *cloud_all_filtered);
						file_cnt++;
						a++;
					}
				}else{    
					//if cloud is not target, leaving only one point to erase all.
					cloud_all_filtered->erase(cloud_all_filtered->begin()+1,cloud_all_filtered->end());
				}

			}else{
				//if cloud is not target, leaving only one point to erase all.
				cloud_all_filtered->erase(cloud_all_filtered->begin()+1,cloud_all_filtered->end());
			}

			if( cloud_all_filtered->points.size() > 1 ){
				pcl::toROSMsg(*cloud_all_filtered, output);
				// pcl::toROSMsg(*cloud_boxel, output);

				//add header to output cloud.
				output.header = input -> header;
				output.header.frame_id = "base_link";

				// Publish the data.
				pub.publish (output);
				is_output = true;
				l++;
			}			
		}	
		std::cout << "PassThrough" << std::endl;
		c++;
	}else{
		z++;
	}


	printf("Output : %d\n", b );
	printf("Output2 : %d\n", l );
	printf("PassThrough : %d\n", c );
	printf("not PassThrough : %d\n", z );
	printf("HighIntensity_count : %d\n", HighIntensity_count );

	//get finish time
	gettimeofday(&f, NULL);
	ROS_INFO_STREAM( "Compute time:" << (f.tv_sec - s.tv_sec) * 1000 + (f.tv_usec - s.tv_usec) / 1000 << "ms" );
	printf("\n");
}


int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "pcl_node");
	ros::NodeHandle nh;

	tf_listener = new tf::TransformListener();

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("hokuyo3d/hokuyo_cloud2", 0, cloud_cb);

	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2> ("output_humansize_cloud", 1);

	// Spin
	ros::spin ();
}
