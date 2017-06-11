/* ROS header files */
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>

/* costum service */
#include <navigation/Path.h>

/* C++ header */
#include <vector>
#include <cmath>
#include <string>

#include <opencv/cv.hpp>
#include <opencv/highgui.h>

/* costum header files */
#include "astar.h"

using namespace std;
using namespace tf;
using namespace cv;

int8_t *map_ = nullptr;
int width = 0, height = 0;
float resolution = 0;

bool map_received = false;


static void inflate_map(float radius){

    if (map_ == nullptr || width <= 0 || height <= 0 || resolution <= 0) return;

    /* find all positions that are occupied */
    std::vector<pos_t> occupied;
    for (auto row = 0; row < height; row++){
        for (auto col = 0; col < width; col++){
            pos_t pos(col,row);
            if(map_[row*width+col] == 100) occupied.push_back(pos);
        }
    }

    ROS_INFO_STREAM("INFLATE MAP ...");

    double angle_increment = atan2(resolution,radius);

    ROS_INFO_STREAM("ANGLE INCREMENT: " << angle_increment);

    for (auto o : occupied){
        for (int r = 1; r < radius/resolution; r++){

            for (double phi = 0; phi < 2* M_PI; phi+= angle_increment){

                int x = r*cos(phi);
                int y = r*sin(phi);

                int pos_x = o.x + x;
                int pos_y = o.y + y;

                if (pos_x >= 0 && pos_x < width && pos_y >= 0 && pos_y < width){
                    map_[pos_y*width + pos_x] = 100;
                }

            }
            //ROS_INFO_STREAM(last_coordinate);
        }
    }

    ROS_INFO_STREAM("DONE");
}

static void map_callback(const nav_msgs::OccupancyGridConstPtr &map){

    if (map_ != nullptr) free(map_);

    width = map->info.width;
    height = map->info.height;
    resolution = map->info.resolution;

    ROS_INFO_STREAM("MAP RECEIVED: " << width << " HEIGHT: " << height << " RESOLUTION: " << resolution);
    map_ = (int8_t *) calloc(width*height, sizeof(int8_t));
    int8_t *map_cpy = (int8_t *) calloc(width*height, sizeof(int8_t));

    if (map_ == nullptr || map_cpy == nullptr){
        ROS_INFO_STREAM("FAILED TO ALLOCATE MEMORY");
        return;
    }

    for (int i = 0; i < width*height; i++){
        map_[i] = map->data[i];

    }

    inflate_map(.15);

    /* copy array since direct opencv access will alter it*/
    for (int i = 0; i < width * height; i++){
        /* unknown cells are interpreted as occupied */
        map_cpy[i] = map_[i] == -1 ?  100 : map_[i];
    }


    /* blur */

    cv::Mat blurr = Mat(height,width,CV_8UC1,map_cpy);
    blur(blurr,blurr,Size(9,9));
    /* copy image back */

    for (auto i = 0; i < width * height; i++){
        map_[i] = map_cpy[i];

    }


    /*
    flip(blurr,blurr,1);
    cv::imshow("blurred",blurr);
    waitKey(0);
    */
    /* free memory */
    free(map_cpy);
    map_received = true;

}

bool get_path_srv(const navigation::Path::Request &req, navigation::Path::Response &resp){

    if(map_ == nullptr || width <= 0 || height <= 0 || resolution <= 0){
        ROS_INFO_STREAM("NO MAP");
        return false;
    }

    ROS_INFO_STREAM("START: " << req.start);
    ROS_INFO_STREAM("GOAL: " << req.goal);
    ROS_INFO_STREAM("WIDHT: " << width << " HEIGHT: " << height);

    // Mat astar_progress(height,width, CV_8UC3, Scalar(0,0,0));
    //Mat astar_progress_hsv;

    /*
    for (int row = 0; row < height; row++){
        for (int col = 0; col < width; col ++){
            if (map_[col*width + row] == 100){
                astar_progress.at<Vec3b>(col,row)[0] = 255;
                astar_progress.at<Vec3b>(col,row)[1] = 255;
                astar_progress.at<Vec3b>(col,row)[2] = 255;
            }
        }
    }
    */

    //  cvtColor(astar_progess,astar_progress_hsv,CV_BGR2HSV);


    Astar astar;
    vector<geometry_msgs::PoseStamped> path;

    astar.set_map(map_,width,height,resolution);

    /* setup integer values for map */
    pos_t start(req.start.x/resolution,req.start.y/resolution);
    pos_t goal(req.goal.x/resolution,req.goal.y/resolution);


    ROS_INFO_STREAM("FIND PATH");
    bool path_found = astar.findPath(start,goal,path);

    if(path_found){

        ROS_INFO_STREAM("PATH FOUND");
        resp.poses = path;

    }
    else ROS_INFO_STREAM("NO PATH FOUND");

    /*
    cv::flip(astar_progress,astar_progress,1);
    imshow("ASTAR PROGRESS",astar_progress);
    waitKey(0);
    */

    return path_found;

}


int main(int argc, char **argv){

    ros::init(argc,argv,"path_service");
    ros::NodeHandle nh;
    ros::Subscriber map_sub = nh.subscribe("/map",1,map_callback);
    ros::ServiceServer server = nh.advertiseService<navigation::Path::Request,navigation::Path::Response>("GetPath",get_path_srv);
    ros::Publisher grid = nh.advertise<nav_msgs::OccupancyGrid>("navigation/occupancygrid",1);
    // ros::spin();


    while(ros::ok()){

        if (map_ != nullptr){

            nav_msgs::OccupancyGrid occ;
            occ.header.frame_id = "map";
            occ.header.stamp = ros::Time::now();
            for(auto i = 0; i < width*height; i++) occ.data.push_back(map_[i]);
            occ.info.height = height;
            occ.info.width = width;
            occ.info.resolution = resolution;
            grid.publish(occ);


            map_received = false;

        }

        ros::spinOnce();
    }


    server.shutdown();

    if (map_ != nullptr) free(map_);

    return 0;

}
