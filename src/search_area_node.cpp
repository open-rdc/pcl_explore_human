#include<ros/ros.h>
#include<tf/transform_listener.h>
#include<pcl_explore_human/Time_Bool.h>
#include<geometry_msgs/PointStamped.h>
#include<nav_msgs/OccupancyGrid.h>
#include<nav_msgs/Odometry.h>

class SearchArea{
    public:
        SearchArea(){
            ros::NodeHandle pnh_("~");
            pnh_.param<std::string>("robot_frame",robot_frame_,"base_link");
            pnh_.param<std::string>("map_frame",map_frame_,"map");
            pnh_.param<std::string>("odom_topic",odom_topic_,"icart_mini/odom");
            pub_= nh_.advertise<geometry_msgs::PointStamped>("map_pose",100);
            pub2_= nh_.advertise<pcl_explore_human::Time_Bool>("area_flag",100);
            sub_= nh_.subscribe<nav_msgs::OccupancyGrid>("map",100,&SearchArea::mapcallback,this);
            sub2_= nh_.subscribe<nav_msgs::Odometry>(odom_topic_,1,&SearchArea::robotposecallback,this);

            tf_listener_ = new tf::TransformListener();
        }
    private:
        void mapcallback(const nav_msgs::OccupancyGridConstPtr &map);
        void robotposecallback(const nav_msgs::OdometryConstPtr &odom);
        int coord_to_cell(const geometry_msgs::PointStamped point);
        bool check_searcharea(const int cell);
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        ros::Subscriber sub2_;
        ros::Publisher pub_;
        ros::Publisher pub2_;

        tf::TransformListener *tf_listener_;

        nav_msgs::OccupancyGrid map_;

        std::string robot_frame_;
        std::string map_frame_;
        std::string odom_topic_;

};

int
SearchArea::coord_to_cell(const geometry_msgs::PointStamped point){
    int cell_x=(map_.info.width/2)+std::round(point.point.x/map_.info.resolution), 
        cell_y=(map_.info.height/2)+std::round(point.point.y/map_.info.resolution);
    return map_.info.width*(cell_y-1)+cell_x+1;
}

bool
SearchArea::check_searcharea(const int cell){
    bool fg=false;

    if(map_.data[cell]==100){
        ROS_DEBUG("occupancy");
        fg=true;
    }
    else if(map_.data[cell]==0){
        ROS_DEBUG("free");
        fg=false;
    }

    else if(map_.data[cell]==-1){
        ROS_DEBUG("unknown");
        fg=false;
    }
    else {
        ROS_WARN("no defined area");
        fg=false;
    }
    return fg;
}

void
SearchArea::mapcallback(const nav_msgs::OccupancyGridConstPtr &map){
    map_.header=map->header;
    map_.info=map->info;
    map_.data=map->data;
}

void
SearchArea::robotposecallback(const nav_msgs::OdometryConstPtr &odom){
    geometry_msgs::PointStamped point;
    geometry_msgs::PointStamped conv_point;
    pcl_explore_human::Time_Bool fg;

    point.header=odom->header;
    point.point=odom->pose.pose.position;

    if(map_.data.empty()){
        ROS_ERROR("no recieved map_server data");
        return;
    }

    try{
		tf_listener_->waitForTransform(map_frame_,odom->header.frame_id,ros::Time(0),ros::Duration(100));
        tf_listener_->transformPoint(map_frame_,point,conv_point);
	}catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
    pub_.publish(conv_point);

    fg.header.stamp=odom->header.stamp;
    fg.data=check_searcharea(coord_to_cell(conv_point));
    ROS_DEBUG("flag_status:%d",fg.data);
    pub2_.publish(fg);

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "search_area_node");
  
  SearchArea sa;

  // Spin
  ros::spin ();
}