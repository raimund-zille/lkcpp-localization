#include "landmarks.h"

Landmarks::Landmarks() : distA(1.), distB(3.){
    updateList();
}

Landmarks::Landmarks(const Landmarks &lm) : distA(lm.distA), distB(lm.distB), lms_(lm.lms_){

}

Landmarks::Landmarks(double a, double b) : distA(a), distB(b){
    updateList();
}

void Landmarks::landmarksInfoStream(){
    for(auto lm : lms_){
        std::string farbe;
        if(lm.specifier == BLUE){
            farbe = "blau";
        }else if(lm.specifier == YELLOW){
            farbe = "gelb";
        }else{
            farbe = "grün";
        }
        ROS_INFO_STREAM("Farbe: " << farbe << "  -  X: " << lm.x << "  -  Y: " << lm.y);
    }
}

double Landmarks::getDistA(){
    return distA;
}

double Landmarks::getDistB(){
    return distB;
}

void Landmarks::setDistA(double a){
    if(a<0){
        a = -a;
    }
    if(a == 0){
        ROS_WARN("Landmarks, length a set to zero!");
    }
    distA = a;
    updateList();
}

void Landmarks::setDistB(double b){
    if(b<0){
        b = -b;
    }
    if(b == 0){
        ROS_WARN("Landmarks, length b set to zero!");
    }
    distB = b;
    updateList();
}

std::vector<landmark> Landmarks::getList(){
    return lms_;
}

landmark Landmarks::createLandmark(Specifier specifier, double x, double y){
    landmark dummy_landmark;
    dummy_landmark.specifier = specifier;
    dummy_landmark.x = x;
    dummy_landmark.y = y;
    return dummy_landmark;
}

void Landmarks::updateList(){
    lms_.clear();

    /* add all landmarks, begin with the blue square, then yellow square, then add all green poles */

    // yellow
    //lms_.push_back(createLandmark(YELLOW, 0.375*distA, 0.5*distB));

    // blue
    //lms_.push_back(createLandmark(BLUE, 2.625*distA, 0.5*distB));

    // green

#if 1
    // even distA (0, a, 2a, 3a)
    for(int i = 0; i < 4; i++){
        lms_.push_back(createLandmark(GREEN, i*distA, 0)); // y = 0
        lms_.push_back(createLandmark(GREEN, i*distA, distB)); // y = distB
    }

    // midpoint
    lms_.push_back(createLandmark(GREEN, 1.5*distA, 0)); // y = 0
    lms_.push_back(createLandmark(GREEN, 1.5*distA, distB)); // y = distB

    // near the edges
    lms_.push_back(createLandmark(GREEN, 0.25*distA, 0)); // y = 0
    lms_.push_back(createLandmark(GREEN, 0.25*distA, distB)); // y = distB
    lms_.push_back(createLandmark(GREEN, 2.75*distA, 0)); // y = 0
    lms_.push_back(createLandmark(GREEN, 2.75*distA, distB)); // y = distB
#endif

#if 0
    //angelina koordinaten
    // even distA (0, a, 2a, 3a)
    for(int i = 0; i < 4; i++){
        lms_.push_back(createLandmark(GREEN, 0, i*distA)); // x = 0
        lms_.push_back(createLandmark(GREEN, distB, i*distA)); // x = distB
    }

    // midpoint
    lms_.push_back(createLandmark(GREEN, 0, 1.5*distA)); // x = 0
    lms_.push_back(createLandmark(GREEN, distB, 1.5*distA)); // x = distB

    // near the edges
    lms_.push_back(createLandmark(GREEN, 0, 0.25*distA)); // x = 0
    lms_.push_back(createLandmark(GREEN, distB, 0.25*distA)); // x = distB
    lms_.push_back(createLandmark(GREEN, 0, 2.75*distA)); // x = 0
    lms_.push_back(createLandmark(GREEN, distB, 2.75*distA)); // x = distB
#endif
}
