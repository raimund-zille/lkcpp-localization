#ifndef LANDMARKS_H
#define LANDMARKS_H

#include "ros/ros.h"
#include <vector>

enum Specifier{YELLOW = 0, BLUE = 100, GREEN = 200};

typedef struct {
    double x;           /// x coordinate
    double y;           /// y coordinate
    Specifier specifier;   /// specifier
} landmark;

class Landmarks{

public:
    /**
     * @brief Landmarks standard constructor
     * a and b will set to default values (1, 3)
     */
    Landmarks();

    /**
     * @brief Landmarks copy constructor
     * @param lm Landmarks which should be copied
     */
    Landmarks(const Landmarks &lm);

    /**
     * @brief Landmarks constructor with known length a and b
     * @param a length a of field
     * @param b length b of field
     */
    Landmarks(double a, double b);

    /**
     * @brief getList returns list of known landmarks
     * @return list of known landmarks
     */
    std::vector<landmark> getList();

    /**
     * @brief setDistA set length a of field
     * @param a length a
     */
    void setDistA(double a);

    /**
     * @brief getDistA return length a of field
     * @return length a of field
     */
    double getDistA();

    /**
     * @brief setDistB set length b of field
     * @param b length b
     */
    void setDistB(double b);

    /**
     * @brief getDistB return lenght b of field
     * @return  length b of field
     */
    double getDistB();

    /**
     * @brief landmarksInfoStream outputs all Landmarks on ROS_INFO_STREAM
     */
    void landmarksInfoStream();


private:
    /**
     * @brief lms_ list of landmarks
     */
    std::vector<landmark> lms_;

    /**
     * @brief distA length a of field
     */
    double distA;

    /**
     * @brief distB length b of field
     */
    double distB;

    /**
     * @brief updateList creates new list of landmarks using distA and distB (call it if at least one of these two values change)
     */
    void updateList();

    /**
     * @brief createLandmark creates landmark with given params
     * @param specifier Specifier of landmark
     * @param x x-Coordinate of landmark
     * @param y y-Coordiante of landmark
     * @return created Landmark
     */
    landmark createLandmark(Specifier specifier, double x, double y);
};



#endif
