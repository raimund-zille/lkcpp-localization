#include "cruisecontrol.h"

using namespace std;


void CruiseControl::odom_cb(const nav_msgs::OdometryConstPtr &odom){

    if(mv_ != nullptr){
        guard_.lock();
        mv_->setOdom(odom);
        guard_.unlock();
    }

    odom_received_ = true;

}

void CruiseControl::wait_for_odom(){

    ROS_WARN_STREAM("WAITING FOR ODOM");
    ros::Rate r(5.);
    double dt = 1./5.;
    double t = 0;
    while(!odom_received_ && ros::ok()){
        if(t += dt > 3./dt) {
            ROS_ERROR_STREAM("NO ODOMETRY AVAIABLE, SHUTDOWN");
            nh_.shutdown();
        }
        ros::spinOnce();
        r.sleep();
    }

    odom_received_ = false;

    ROS_WARN_STREAM("DONE");


}

CruiseControl::CruiseControl(ros::NodeHandle &nh, const std::string &odom_sub, const string &twist_topic, double dt):
    nh_(nh), dt_(dt),\
    run_thread_(false), execute_thread_(false),\
    guard_(), odom_received_(false)


{
    odom_sub_ = nh_.subscribe(odom_sub,1,&CruiseControl::odom_cb,this);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>(twist_topic,10);

    mv_ = nullptr;

    wait_for_odom();

    turn_ = new Turn();
    mbda_ = new MoveByDistAngle();
    mtp_ = new MoveToPos();
    mwo_ = new MoveWithOdom();

    run_thread_ = true;
    thr = new boost::thread(boost::bind(&CruiseControl::run, this));

    start();

}

CruiseControl::~CruiseControl(){

    stop();
    mv_ = nullptr;
    delete turn_;
    delete mbda_;
    delete mtp_;
    delete mwo_;
    delete thr;

}

void CruiseControl::start(double dt){

    /* thread sleeps in ms */
    dt_ = dt*1000;
    execute_thread_ = true;

}

void CruiseControl::stop(){
    execute_thread_ = false;
}



bool CruiseControl::turn(double angle, double thresh){

    bool success;

    /* wait until control was gained */
    guard_.lock();

    /* update turn values */
    success = turn_->turnBy(angle,thresh);

    /* execute turn in next thread iteration */
    mv_ = turn_;

    /* return control to thread */
    guard_.unlock();

    /* reset all mbda_, mtp_ and mwo_ values */
    delete mbda_;
    delete mtp_;
    delete mwo_;
    mbda_ = new MoveByDistAngle();
    mtp_ = new MoveToPos();
    mwo_ = new MoveWithOdom();
    return success;


}

bool CruiseControl::moveTo(double dist, double angle_diff, double thresh){

    bool success;

    /* wait until control was gained */
    guard_.lock();

    /* update move... values */
    success = mbda_->moveByDistAngle(dist,angle_diff,thresh);

    /* execute moveTo in next thread iteration */
    mv_ = mbda_;

    /* return control to thread */
    guard_.unlock();

    /* reset all turn_ values */
    delete turn_;
    delete mtp_;
    delete mwo_;
    turn_ = new Turn();
    mtp_ = new MoveToPos();
    mwo_ = new MoveWithOdom();

    return success;

}

bool CruiseControl::moveTo(const pose_t &start, const pose_t &goal, double thresh){

    bool success;

    /* wait until control was gained */
    guard_.lock();

    /* update move... values */
    success = mtp_->moveFromPosToGoal(start,goal,thresh);

    /* execute moveTo in next thread iteration */
    mv_ = mtp_;

    /* return control to thread */
    guard_.unlock();

    /* reset all turn_, mbda_ and mwo_ values */
    delete turn_;
    delete mbda_;
    delete mwo_;
    turn_ = new Turn();
    mbda_ = new MoveByDistAngle();
    mwo_ = new MoveWithOdom();

    return success;

}

bool CruiseControl::moveToOdomPos(const Vector2D &goal, double thresh){

    bool success;

    /* wait until control was gained */
    guard_.lock();

    /* update move... values */
    success = mwo_->moveToOdomPos(goal,thresh);

    /* execute moveToOdomPos in next thread iteration */
    mv_ = mwo_;

    /* return control to thread */
    guard_.unlock();

    /* reset all turn_, mbda_ and mwo_ values */
    delete turn_;
    delete mbda_;
    delete mtp_;
    turn_ = new Turn();
    mbda_ = new MoveByDistAngle();
    mtp_ = new MoveToPos();

    return success;

}


bool CruiseControl::reachedGoal(){
    if(mv_ != nullptr) return mv_->reachedGoal();
    else return false;
}

void CruiseControl::run(){

    auto last_time = ros::Time::now();
    bool first_time_measurement = true;

    ROS_INFO_STREAM("STARTING THREAD");

    while(run_thread_){

        if (mv_ != nullptr && odom_received_ && execute_thread_){


            auto current = ros::Time::now();
            auto time_diff = current - last_time;
            last_time = current;
            double dt = first_time_measurement ? (first_time_measurement = false, 0.) : time_diff.toSec();


            /* entering critical section */
            guard_.lock();

            if (!first_time_measurement){
                mv_->move(dt,twist_pub_);
                odom_received_ = false;
            }

            /* leave critical section */
            guard_.unlock();

        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(dt_));
    }

    ROS_INFO_STREAM("TERMINATE THREAD");
    thr->join();
}

