#include<ros/ros.h>
#include <tf/transform_listener.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/Point.h>
#include<nav_msgs/OccupancyGrid.h>
#include<cmath>

class BinaryBayse{
    public:
        BinaryBayse(){
            map_flag_=false;
            ros::NodeHandle pnh_("~");
            pnh_.param<float>("resolution",map_.info.resolution,0.20);
            pnh_.param<double>("target_prob",target_prob_,0.55);
            pnh_.param<double>("target_thresh",target_thresh_,0.90);
            pub_=nh_.advertise<geometry_msgs::PointStamped>("filtered_target_point",1);
            sub_=nh_.subscribe<geometry_msgs::PointStamped>("target_point",1,&BinaryBayse::pointcallback,this);
            sub2_=nh_.subscribe<nav_msgs::OccupancyGrid>("map_for_searcharea",100,&BinaryBayse::mapcallback,this);
            tf_listener_ = new tf::TransformListener();
        }

    private:
        void pointcallback(const geometry_msgs::PointStampedConstPtr &target_point);
        void mapcallback(const nav_msgs::OccupancyGridConstPtr &map);
        int coord_to_cell(const double x,double y);
        double prob_to_exp(const double prob);
        double exp_to_prob(const double exp);
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        ros::Subscriber sub2_;
        ros::Publisher pub_;

        tf::TransformListener *tf_listener_;

        std::string robot_frame_;
        std::string map_frame_;

        geometry_msgs::Point point;
        nav_msgs::OccupancyGrid map_;
        std::vector<double> target_cell_;
        double target_thresh_;
        double target_prob_;
        bool map_flag_;
};

int
BinaryBayse::coord_to_cell(const double x,double y){
    int cell_x=(map_.info.width/2)+std::round(x/map_.info.resolution), 
        cell_y=(map_.info.height/2)+std::round(y/map_.info.resolution);
    return map_.info.width*(cell_y-1)+cell_x+1;
}

void
BinaryBayse::mapcallback(const nav_msgs::OccupancyGridConstPtr &map){
    double resolution_rate=map->info.resolution/map_.info.resolution;
    map_.header=map->header;
    map_.info.origin.position.x=map->info.origin.position.x*resolution_rate;
    map_.info.origin.position.y=map->info.origin.position.y*resolution_rate;
    map_.info.origin.position.z=map->info.origin.position.z*resolution_rate;
    map_.info.height=map->info.height*resolution_rate;
    map_.info.width=map->info.width*resolution_rate;
    target_cell_.resize(map_.info.height*map_.info.width);
    std::cout<<resolution_rate<<std::endl;
    std::cout<<map_.info.origin.position.x<<std::endl;
    std::cout<<map_.info.height<<std::endl;
    std::cout<<map_.info.width<<std::endl;
    std::cout<<target_cell_.size()<<std::endl;
    map_flag_=true;
}

double
BinaryBayse::prob_to_exp(const double prob){
    return std::log(prob/(1-prob));
}

double
BinaryBayse::exp_to_prob(const double exp){
    return 1/(1+std::exp(-exp));
}

void 
BinaryBayse::pointcallback(const geometry_msgs::PointStampedConstPtr &target_point){
    geometry_msgs::PointStamped filtered_target_point;
    if(!map_flag_){
        ROS_ERROR("no map received");
        return;
    }

    int cell=coord_to_cell(target_point->point.x,target_point->point.y);

    target_cell_[cell]+=prob_to_exp(target_prob_);

    ROS_INFO("cell:%d, prob:%lf",cell,exp_to_prob(target_cell_[cell]));

    if(exp_to_prob(target_cell_[cell])>=target_thresh_){
        ROS_INFO("find_target");
        filtered_target_point=*target_point;
        std::cout<<filtered_target_point.point.x<<std::endl;
        pub_.publish(filtered_target_point);
        target_cell_[cell]=0;
    }

}

int main(int argc, char **argv){
    ros::init(argc,argv,"binary_bayse");

    BinaryBayse bb;

    ros::spin();
}