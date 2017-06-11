#ifndef MEASUREMENT_MODEL_H
#define MEASUREMENT_MODEL_H

#include "particle.h"
#include <vector>
#include "landmarks.h"

#define STDDEV_R   0.01
#define STDDEV_PHI 0.017
#define STDDEV_IDF 0.005
#define SQRT_2_PI  2.5066282746310002

typedef struct {
    double r;           /// distance to feature
    double phi;         /// angular to feature
    double specifier;   /// specifier to feature
    int knownCorr;      /// Index of landmark if clearly (blue, yellow squares) -1 if unknown(green)
} feature;


class MeasurementModel{

public:

    MeasurementModel(const Landmarks &lm): lm_(lm){
        lms_ = lm_.getList();
    }
    /**
     * @brief get_weight calculate weight of Particle with the given features f
     * @param pos position of robot
     * @param f list of observed features
     * @return sum of corresponding probabilty from landmarks and features divided by the number of observed features f
     */
    double get_weight(const pose_t &pos, const std::vector<feature> &f, bool debug = false);
    void test(double a, double b);

protected:

    Landmarks lm_;
    std::vector<landmark> lms_;


};

/*
class LandmarkKnownCorr : public MeasurementModel {

public:

    LandmarkKnownCorr(const std::vector<landmark> &lm):MeasurementModel(lm){}
    double get_weight(const pose_t &pos, const feature &f);

private:


    const landmark &getCorrFeature();
};

*/
/*
class LandMarkUnknownCorr : public MeasurementModel {

public:

    LandMarkUnknownCorr(const std::vector<landmark> &lm):MeasurementModel(lm){}
    double get_weight(const pose_t &pos, const std::vector<feature> &f);

};
*/

#endif
