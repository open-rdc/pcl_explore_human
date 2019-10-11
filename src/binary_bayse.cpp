#include<ros/ros.h>
#include <tf/transform_listener.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/Point.h>
#include<jsk_recognition_msgs/SimpleOccupancyGridArray.h>
#include<cmath>

class BinaryBayse{
    public:
        BinaryBayse(){
            sub_=nh_.subscribe<geometry_msgs::PointStamped>("target_point",1,&BinaryBayse::callback,this);
            pub2_=nh_.advertise<jsk_recognition_msgs::SimpleOccupancyGridArray>("simple_grid",1);
            tf_listener_ = new tf::TransformListener();
            matrix.resize(nx_,std::vector<double>(nx_));
        }

    private:
        void callback(const geometry_msgs::PointStampedConstPtr &target_point);
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        ros::Publisher pub_;
        ros::Publisher pub2_;

        tf::TransformListener *tf_listener_;

        jsk_recognition_msgs::SimpleOccupancyGridArray grid_array;
        jsk_recognition_msgs::SimpleOccupancyGrid grid;
        geometry_msgs::Point point;

        double cell_size_=0.2;
        double search_range_ = 3.5;
        int nx_= std::round(search_range_/ cell_size_)*2;
        std::vector<std::vector<double>> matrix;

        double target_prob_ = 0.55;
        double decay_prob_ = 0.45;
};

void 
BinaryBayse::callback(const geometry_msgs::PointStampedConstPtr &target_point){
    geometry_msgs::PointStamped trans_point;
    geometry_msgs::PointStamped conv_point;

    trans_point.header.frame_id=target_point->header.frame_id;
    trans_point.point.x=target_point->point.x;
    trans_point.point.y=target_point->point.y;

	try{
		tf_listener_->waitForTransform("base_link", target_point->header.frame_id, ros::Time(0), ros::Duration(100.0));
	}catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
    /*
    std::cout<<"target_point"<<std::endl;
    std::cout<<target_point->point.x<<std::endl;
    std::cout<<target_point->point.y<<std::endl;
    */
    try{
	    tf_listener_->transformPoint("base_link",trans_point, conv_point);
		}
		catch(tf::TransformException &e){
			ROS_WARN_STREAM("tf::TransformException: " << e.what());
		}
    
    std::cout<<"conv_point"<<std::endl;
    std::cout<<conv_point.point.x<<std::endl;
    std::cout<<conv_point.point.y<<std::endl;
    
    int x_cell = std::round(conv_point.point.x/cell_size_+(search_range_/cell_size_));
    int y_cell = std::round(conv_point.point.y/cell_size_+(search_range_/cell_size_));

    matrix[x_cell][y_cell]+=std::log(target_prob_/(1-target_prob_));
    //matrix[x_cell][y_cell]+=1;
    //std::cout<< matrix[x_cell][y_cell]<<std::endl;

    for(int i=0;i<nx_;i++){
        for(int j=0;j<nx_;j++){
            //matrix[i][j] += std::log(decay_prob_/(1-decay_prob_));

            if(1/(1+exp(-matrix[i][j]))>0.9){

                //std::cout<<matrix[i][j]<<std::endl;
                std::cout<<"x:"<<i<<" y:"<<j<<" prob:"<<1/(1+exp(-matrix[i][j]))<<std::endl;
                //std::cout<<"x:"<<i<<" y:"<<j<<" value:"<<matrix[i][j]<<std::endl;

                double x = (i-(search_range_/cell_size_))*cell_size_;
                double y = (j-(search_range_/cell_size_))*cell_size_;

                point.x=x;
                point.y=y;
                point.z=0;
                grid_array.header.frame_id="base_link";
                grid.header.frame_id="base_link";
                grid.resolution=0.2;
                grid.coefficients[0]=0;
                grid.coefficients[1]=0;
                grid.coefficients[2]=0;
                grid.coefficients[3]=0;
                grid.cells.push_back(point);
                grid_array.grids.push_back(grid);
                pub2_.publish(grid_array);


                std::cout<<"x y"<<std::endl;
                std::cout<<x<<std::endl;
                std::cout<<y<<std::endl;

            }
        }
    }

}

int main(int argc, char **argv){
    ros::init(argc,argv,"binary_bayse");

    BinaryBayse bb;

    ros::spin();
}