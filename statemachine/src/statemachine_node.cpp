#include <ros/ros.h>
#include "GlobalStateMachine.h"

#include <QtGui>
#include <QApplication>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sm_node");
    ros::NodeHandle nh;

    QApplication app(argc, argv);

    GlobalStateMachine sm(nh);

    return app.exec();
}

