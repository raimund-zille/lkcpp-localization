#include <geometry_msgs/Twist.h>

#include "roboControl.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <angles/angles.h>
#include "util.h"
#include "math.h"
/* compile test */
MoveWithOdom test();


RoboControl::RoboControl(ObjectParser* objp) : odomReceived_(false) , rate_(50)
{
    ROS_INFO_STREAM("RoboControl initialised!");
    keepMoving = MOVING::stop;
    objParser_ = objp;

    // Advertise a new publisher for the simulated robot's velocity command topic
    commandPub = node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 100);
    poseSub = node.subscribe("/odom", 40, &RoboControl::odomCallback, this);
    scanSub = node.subscribe("/laserscan", 10, &RoboControl::laserCallback, this);
    ros::spinOnce();
    rate_.sleep();
}

void RoboControl::turn(TURN direction)
{
    ROS_INFO_STREAM("Send one turn message ");
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.angular.z = direction == TURN::left ? -0.8 : direction == TURN::right ? 0.8 : 0;
    commandPub.publish(msg);
}

void RoboControl::turnDegree(double degrees)
{
    ROS_INFO_STREAM("Turn robot " << degrees << " degrees.");

    while(!odomReceived_)
        ros::spinOnce();

    if (degrees < -180 || degrees > 180)
    {
        ROS_ERROR_STREAM("RoboControl::Turn: degrees must be in range(-180,180)");
        return;
    }

    TURN direction = degrees > 0 ? TURN::left : TURN::right;
    degrees = fabs(degrees);

    double travelled = 0;
    double lastAngle = currAngle_;

    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.angular.z = direction == TURN::left ? -0.4 : direction == TURN::right ? 0.4 : 0;

    while(node.ok())
    {
        commandPub.publish(msg);
        odomReceived_ = false;
        while (!odomReceived_)
        {
            ros::spinOnce();
            rate_.sleep();
        }

        travelled += angles::to_degrees(fabs(angles::shortest_angular_distance(currAngle_, lastAngle)));
        lastAngle = currAngle_;

        if (travelled > degrees)
        {
            ROS_INFO_STREAM("Turn robot " << degrees << " degrees: Done.");
            finished_ = true;
            return;
            break;
        }
    }
    ROS_INFO_STREAM("Turn robot " << degrees << " degrees: Done.");
    return;
}

void RoboControl::laserCallback(const sensor_msgs::LaserScanConstPtr &scan){
    //check if any laserscan is below certain threshold
    for(int i = 0; i < scan->ranges.size(); i++){
        if(scan->ranges[i] < CHECK_LASER){
            double angle = scan->angle_min + i*scan->angle_increment + M_PI;
            double r = scan->ranges[i];
            if(checkPhiR(angle, r)){
                wayBlocked_ = true;
                return;
            }
        }
    }
    wayBlocked_ = false;
}

bool RoboControl::checkPhiR(double phi, double r){
    geometry_msgs::Pose obj_pose = currPos_;
    double obj_angle = currAngle_ + phi;
    obj_pose.position.x += r*cos(obj_angle);
    obj_pose.position.y += r*sin(obj_angle);

    BlobArray curr = objParser_->bluePucks;
    if(checkBlos(curr, obj_pose)){
        return false;
    }

    curr = objParser_->yellowPucks;
    if(checkBlos(curr, obj_pose)){
        return false;
    }

    std::vector<Blob> currB = objParser_->greenPosts;
    curr.blobs = currB;
    if(checkBlos(curr, obj_pose)){
        return false;
    }
    blocked_r_ = r;
    blocked_phi_ = phi;
    return true;

}

bool RoboControl::checkBlos(BlobArray blobs, geometry_msgs::Pose pos){
    for(size_t i = 0; i < blobs.blobs.size(); i++){
        Blob cb = blobs.blobs[i];
        if(check_dist(cb, pos)<THRESHOLD_DIST){
            return true;
        }
    }
    return false;
}

double RoboControl::check_dist(Blob blob, geometry_msgs::Pose pos){
    double x_diff = pos.position.x - blob.worldPosition.position.x;
    double y_diff = pos.position.y - blob.worldPosition.position.y;

    double diff = sqrt(pow(x_diff,2)+pow(y_diff,2));
    return diff;
}

void RoboControl::r_phiRad_from_pos(double x_me, double y_me, double phi_me, double x_des, double y_des, double &r, double &phi){
    //phi_me in RAD!!!! phi ist auch in RAD!!!
    double x_diff = x_des - x_me;
    double y_diff = y_des - y_me;
    r = sqrt(pow(x_diff,2)+pow(y_diff,2));
    phi = atan2(y_diff, x_diff) + phi_me;
    while(phi > M_PI_2){
        phi -= 2*M_PI_2;
    }
    while(phi < M_PI_2){
        phi += 2*M_PI_2;
    }
}


// Send a velocity command
void RoboControl::move(double angleSpeed, double speed) {
       geometry_msgs::Twist msg; // The default constructor will set all commands to 0
       msg.angular.z = angleSpeed;
       msg.linear.x = speed;
       commandPub.publish(msg);
}

bool RoboControl::move(Blob &b, double speed) {
       geometry_msgs::Twist msg; // The default constructor will set all commands to 0
       // ROS_INFO_STREAM("drive");
       tf::Quaternion q;
       tf::quaternionMsgToTF(b.pose.orientation, q);
       tf::Matrix3x3 m(q);
       double roll, pitch, yaw;
       m.getRPY(roll, pitch, yaw);
       double aimAngle = angles::normalize_angle(yaw);
       aimAngle += angles::from_degrees(b.angleDegree);

       double diff = angles::shortest_angular_distance(currAngle_, aimAngle);
       msg.angular.z = 0.7 * diff;
       ROS_INFO_STREAM("ANGLE: " << angles::to_degrees(diff));
       msg.linear.x = speed;
       commandPub.publish(msg);
       return false;
}

bool RoboControl::move(Vector2d aim, double pControl, double holdDistance, bool useAvoid, bool useBackwards) {
    int (*sign)(const double&) = &boost::math::sign<double>;
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    // ROS_INFO_STREAM("drive");
    lineToGoal_ = LineSegment(currLine_.getSupportVector(), aim - currLine_.getSupportVector());
    double diffAngle = (aim - currLine_.getSupportVector()).getAngle(currLine_.getDirectionVector());
    double distance = (aim - currLine_.getSupportVector()).getLength();

    bool driveBackwards = false;
    if (fabs(diffAngle) > M_PI/2)
    {
        driveBackwards = true;
    }
    else
    {
        driveBackwards = false;
    }
    if (useBackwards == false) driveBackwards = false;
    if (driveBackwards == true) {
        diffAngle = diffAngle - sign(diffAngle) * M_PI ;
    }

    msg.angular.z = diffAngle * 1.35;
    if (fabs(diffAngle) < M_PI_4/2) {
        msg.linear.x = 0.09 + pControl * 0.5;
        if (driveBackwards) {
            msg.linear.x = -0.09 - pControl * 0.5;
        }
    }

    ROS_INFO_STREAM("Dist " << distance << " " << diffAngle << " " << aim << " " << currPos_);
//    move_base_.moveToOdomPos(Vector2D(aim.x, aim.y), holdDistance);
//    move_base_.angleCtrl_.set_new_params(4.,0,0,-0.6,0.6);
//    move_base_.velCtrl_.set_new_params(pControl, 0, 0, 0.01, 1.);
//    if(pControl!= 0 && useAvoid){
//        if(wayBlocked_){
//            if(fabs(blocked_phi_)<M_PI_4){
//                double dir = -0.5;
//                if(blocked_phi_>0){
//                    dir *= -1;
//                }
//                move(dir, 0);
//                return false;
//            }
//        }
//    }
//    ROS_INFO_STREAM("Distance " << distance);
//    move_base_.move(0.1, commandPub);
   // ROS_INFO_STREAM("ANGLE: " << angles::to_degrees(diff) << "  Distance: " << distance);
//    if (distance < holdDistance) {
//        return true;
//    }
    commandPub.publish(msg);
    if (distance < holdDistance) {
        return true;
    } else  {
        return false;
    }
    //return move_base_.reachedGoal();
}
bool RoboControl::turn(Vector2d aim) {
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    // ROS_INFO_STREAM("drive");
    lineToGoal_ = LineSegment(currLine_.getSupportVector(), aim - currLine_.getSupportVector());
    double diffAngle = (aim - currLine_.getSupportVector()).getAngle(currLine_.getDirectionVector());
    double distance = (aim - currLine_.getSupportVector()).getLength();

    msg.angular.z = diffAngle * 1.35;
    ROS_INFO_STREAM("Dist " << distance << " " << diffAngle << " " << aim << " " << currPos_);
    commandPub.publish(msg);
    if (fabs(diffAngle) < 0.1) {
        return true;
    } else  {
        return false;
    }
}

bool RoboControl::move_by_dist() {
    move_by_dist_.move(0.1, commandPub);
    return move_by_dist_.reachedGoal();
}

bool RoboControl::driveBackToHome() {
       geometry_msgs::Twist msg; // The default constructor will set all commands to 0
       // ROS_INFO_STREAM("drive");
       tf::Quaternion q;
       tf::quaternionMsgToTF(poseInit_.orientation, q);
       tf::Matrix3x3 m(q);
       double roll, pitch, yaw;
       m.getRPY(roll, pitch, yaw);
       double aimAngle = angles::normalize_angle(yaw);
       //aimAngle += angles::from_degrees(b.angleDegree);


       double diff = angleBetweenTwoPoints(poseInit_, currPos_);
       diff -= currAngle_;
      // ROS_INFO_STREAM("DIFF " << diff << " c" << currPos_ << " pI " << poseInit_);
               //angles::shortest_angular_distance(currAngle_, aimAngle);
       if (fabs(diff) > M_PI_2) {
           msg.angular.z = 0.5 * diff;
       } else {
           msg.angular.z = 0.5 * diff;
           msg.linear.x = 0.14;
       }
       commandPub.publish(msg);
       double dist = distanceBetweenTwoPoints(currPos_, poseInit_);
       if (dist < 0.2) {
           return true;
       } else {
           return false;
       }
}

bool RoboControl::driveToGoal() {
       geometry_msgs::Twist msg; // The default constructor will set all commands to 0
       // ROS_INFO_STREAM("drive");
//       tf::Quaternion q;
//       tf::quaternionMsgToTF(poseInit_.orientation, q);
//       tf::Matrix3x3 m(q);
//       double roll, pitch, yaw;
//       m.getRPY(roll, pitch, yaw);
//       double aimAngle = angles::normalize_angle(yaw);
       //aimAngle += angles::from_degrees(b.angleDegree);


       double diff = angleBetweenTwoPoints(poseAim_, currPos_);
       double dist = distanceBetweenTwoPoints(poseAim_, currPos_);
       diff -= currAngle_;
       //ROS_INFO_STREAM("DIFF " << diff << " and dist " << dist);
               //angles::shortest_angular_distance(currAngle_, aimAngle);
       if (fabs(diff) > M_PI_2) {
           msg.angular.z = 0.5 * diff;
       } else {
           msg.angular.z = 0.5 * diff;
           msg.linear.x = 0.14;
       }
       commandPub.publish(msg);

       if (dist < 0.2) {
           return true;
       } else {
           return false;
       }
}


void RoboControl::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    move_base_.setOdom(msg);
    turn_base_.setOdom(msg);
    move_by_dist_.setOdom(msg);
    static bool firstRun = false;
    if (firstRun == false) {
        ROS_INFO_STREAM("INIT FIRST RUN");
        firstRun = true;
        poseInit_ = msg->pose.pose;
        tf::Quaternion q;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        yawInit_ = yaw;
        poseAim_.position.x = poseInit_.position.x + 2.4;
        poseAim_.position.y = poseInit_.position.y;
    }
    currPos_  = msg->pose.pose;

    // angles
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    currAngle_ = angles::normalize_angle(yaw);
    currLine_ = Line(currPos_, Vector2d(currAngle_));


    odomReceived_ = true;
}

void RoboControl::setAngleFromOdom(TURN direction)
{
    odomReceived_ = false;
    while (!odomReceived_)
    {
        ros::spinOnce();
        rate_.sleep();
    }
    angleSet_ = currAngle_;
    direction_ = direction;
}

double RoboControl::getTurnedAngle()
{
    odomReceived_ = false;
    while (!odomReceived_)
    {
        ros::spinOnce();
        rate_.sleep();
    }
    double turnedAngle = 0;
    if (direction_ == TURN::left)
    {
        turnedAngle = angles::shortest_angular_distance(angleSet_, currAngle_);
    }
    return 0;
}

double RoboControl::getMeanDistanceTo(std::vector<Blob> &blobs) {
    double distance = 0;
    for(Blob &b : blobs) {
        distance += LineSegment(currPos_, b.worldPosition).getLength();
    }
    if (distance < 0.001) return 10;
    return distance / blobs.size();
}

double RoboControl::getClosestDistanceTo(std::vector<Blob> &blobs) {
    double distance = 10;
    Blob *used;
    for(Blob &b : blobs) {
        double dist =  LineSegment(currPos_, b.worldPosition).getLength();
        if (dist < distance) {
            distance = dist;
            used = &b;
        }
    }
    if (distance < 9) used->alreadyUsedToDrive = true;
    return distance;
}

double RoboControl::getClosestDistanceToUnused(std::vector<Blob> &blobs) {
    double distance = 10;
    Blob *used;
    for(Blob &b : blobs) {
        double dist =  LineSegment(currPos_, b.worldPosition).getLength();
        if ((dist < distance) && b.alreadyUsedToDrive == false) {
            distance = dist;
        }
    }
    if (distance < 9) used->alreadyUsedToDrive = true;
    return distance;
}



