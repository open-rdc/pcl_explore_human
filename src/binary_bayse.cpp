#include<ros/ros.h>
#include <tf/transform_listener.h>
#include<geometry_msgs/PointStamped.h>

#include<cmath>

class BinaryBayse{
    public:
        BinaryBayse(){
            sub_=nh_.subscribe<geometry_msgs::PointStamped>("target_point",1,&BinaryBayse::callback,this);
            tf_listener_ = new tf::TransformListener();
        }

    private:
        void callback(const geometry_msgs::PointStampedConstPtr &target_point);
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        ros::Publisher pub_;

        tf::TransformListener *tf_listener_;

        double cell_size_=0.2;
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
    std::cout<<"target_point"<<std::endl;
    std::cout<<target_point->point.x<<std::endl;
    std::cout<<target_point->point.y<<std::endl;

    try{
	    tf_listener_->transformPoint("base_link",trans_point, conv_point);
		}
		catch(tf::TransformException &e){
			ROS_WARN_STREAM("tf::TransformException: " << e.what());
		}

    std::cout<<"conv_point"<<std::endl;
    std::cout<<conv_point.point.x<<std::endl;
    std::cout<<conv_point.point.y<<std::endl;
    
    std::cout<<"x_cell:"<<(std::round(conv_point.point.x*100)/100)/cell_size_<<std::endl;
    std::cout<<"y_cell:"<<(std::round(conv_point.point.y*100)/100)/cell_size_<<std::endl;

    
  
}

int main(int argc, char **argv){
    ros::init(argc,argv,"binary_bayse");

    BinaryBayse bb;

    ros::spin();
}