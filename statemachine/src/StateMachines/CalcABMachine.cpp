#include "CalcABMachine.h"
#include "ObjectParser.h"
#include "image_processing.h"
#include <algorithm>
using namespace std;

double CalcABMachine::createAB(std::vector<Blob> &t1_sorted, std::vector<Blob> &t2_sorted) {
    vector<double> distances;
    for(int i = 0; i < t1_sorted.size() - 1; i++) {
        for(int j = i + 1; j < t1_sorted.size(); j++) {
            distances.push_back(distanceBetweenTwoPoints(t1_sorted[i].worldPosition , t1_sorted[j].worldPosition));
        }
    }
    sort(distances.begin(), distances.end());
    double factor[] = { 0.25, 0.5, 0.75, 1, 1.25, 1.5 };
    for(int i = 0; i < distances.size(); i++) {
        a += factor[i] * distances[i];
        ROS_INFO_STREAM("dist : " << factor[i] <<"  d" << distances[i]);
    }
    a/=distances.size();
    ROS_INFO_STREAM("FINAL a: " << a);
    vector<vector<double>> dist;
    for(int i = 0; i < t1_sorted.size() ; i++) {
        vector<double> d;
        for(int j = 0; j < t1_sorted.size(); j++) {
            if (j == i) continue;
            d.push_back(distanceBetweenTwoPoints(t1_sorted[i].worldPosition , t1_sorted[j].worldPosition));
        }
        dist.push_back(d);
    }

    distances.clear();
    for(int i = 0; i < t2_sorted.size(); i++) {
        double closest = 10;
        for(int j = 0; j < t2_sorted.size(); j++) {
            double dist = distanceBetweenTwoPoints(t2_sorted[i].worldPosition , t2_sorted[j].worldPosition);
            if (dist < closest) {
                closest = dist;
            }
            if (j > i) {
                distances.push_back(dist);
            }
        }
        t2_sorted[i].distanceToNextClosestGreenPost = closest;
    }
    sort(distances.begin(), distances.end());
    double a2 = 0;
    for(int i = 0; i < distances.size(); i++) {
        a2 += factor[i] * distances[i];
        ROS_INFO_STREAM("dist : " << distances[i]);
    }
    a2/=distances.size();
   // ROS_INFO_STREAM("FINAL a: " << a2);
    a = (a + a2) / 2;
  //  ROS_INFO_STREAM("FINALCombined a: " << a);
//    double a = 0;
//    double dist1 = distanceBetweenTwoPoints(greenPostsSide1[0].worldPosition , greenPostsSide1[1].worldPosition);
//    double dist2 = distanceBetweenTwoPoints(greenPostsSide1[0].worldPosition, greenPostsSide1[2].worldPosition);
//    double dist3 = distanceBetweenTwoPoints(greenPostsSide1[0].worldPosition, greenPostsSide1[3].worldPosition);

//    double ang1 = angleBetweenTwoPoints(greenPostsSide1[0].worldPosition ,greenPostsSide1[1].worldPosition);
//    double ang2 = angleBetweenTwoPoints(greenPostsSide1[0].worldPosition, greenPostsSide1[2].worldPosition);
//    double ang3 = angleBetweenTwoPoints(greenPostsSide1[0].worldPosition, greenPostsSide1[3].worldPosition);

//    ROS_INFO_STREAM(" a:" << rect.size.width << "  " << rect.size.height << "  " << dist3);
//    ROS_INFO_STREAM(" SIDE 1 ang: " << ang1 << "  " << ang2 << "  " << ang3);
//    dist1 = distanceBetweenTwoPoints(greenPostsSide2[0].worldPosition, greenPostsSide2[1].worldPosition);
//    dist2 = distanceBetweenTwoPoints(greenPostsSide2[0].worldPosition, greenPostsSide2[2].worldPosition);
//    dist3 = distanceBetweenTwoPoints(greenPostsSide2[0].worldPosition, greenPostsSide2[3].worldPosition);
//    ang1 = angleBetweenTwoPoints(greenPostsSide2[0].worldPosition ,greenPostsSide2[0].worldPosition,  greenPostsSide2[1].worldPosition);
//    ang2 = angleBetweenTwoPoints(greenPostsSide2[0].worldPosition,greenPostsSide2[0].worldPosition,  greenPostsSide2[2].worldPosition);
//    ang3 = angleBetweenTwoPoints(greenPostsSide2[0].worldPosition,greenPostsSide2[0].worldPosition,  greenPostsSide2[3].worldPosition);
//    ROS_INFO_STREAM(" SIDE 2 dist: " << dist1 << "  " << dist2 << "  " << dist3);
//    ROS_INFO_STREAM(" SIDE 2 ang: " << ang1 << "  " << ang2 << "  " << ang3 );
//    greenPosts.clear();
//    greenPosts.insert(greenPosts.end(), t1_sorted.begin(), t1_sorted.end());
//    greenPosts.insert(greenPosts.end(), t2_sorted.begin(), t2_sorted.end());

    distances.clear();
    for(int i = 0; i < t1_sorted.size(); i++) {
        for(int j = 0; j < t2_sorted.size(); j++) {
            distances.push_back(distanceBetweenTwoPoints(t1_sorted[i].worldPosition , t2_sorted[j].worldPosition));
        }
    }
    sort(distances.begin(), distances.end());
    for(int i = 0; i < distances.size(); i++) {
       // ROS_INFO_STREAM("B - dist : " << distances[i]);
    }

    b = (distances[0] + distances[1] + distances[2] + distances[3]) / 4;
    a_b = a/b;
    ROS_INFO_STREAM("A : " << a);
    ROS_INFO_STREAM("B : " << b);
    ROS_INFO_STREAM("A/B : " << a/b);
    return a_b;
}
