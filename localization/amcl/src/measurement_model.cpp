#include "measurement_model.h"

static double getProbGauss(const double &diff, const double &stddev){
    // diff: Wert - mittelwert
    // stddev: Standdardabweichung

    double denom = sqrt(stddev)*SQRT_2_PI;
    double expo = -0.5*pow(diff,2)/stddev;
    double prob = (1./denom)*exp(expo);

    //std::cout <<"probGauss: " << "diff: " << diff << " stddev: " << stddev << " prob: " <<prob << std::endl;
    return prob;
}

#if 0
static double getCorrProb(const pose_t &pos, const feature &f, const landmark &lm ){


   /* compare landmark to sampled pose */
   double x_diff_prime = lm.x - pos.x;
   double y_diff_prime = lm.y - pos.y;
   double phi_prime = atan2(y_diff_prime, x_diff_prime);
   double r_prime = sqrt(x_diff_prime*x_diff_prime + y_diff_prime*y_diff_prime);

   return getProbGauss(f.r-r_prime, STDDEV_R)*getProbGauss(f.phi-phi_prime, STDDEV_PHI);

}
#endif
#if 1

/**
 * @brief getCorrProb returns probability
 * @param pos my position
 * @param f detected feature
 * @param lm correlated landmark
 * @param debug if debug prints should be made
 * @return probability
 */
static double getCorrProb(const pose_t &pos, const feature &f, const landmark &lm , bool debug = false){

    //Can Test: dont compare r, phi, compare x,y x_des = pos_x+cos(f.phi+pos_theta)*r NOT CORRECT WAY BECAUSE YOU GET BOTH MEASUREMENTS ERROR IN x_des and y_des

    double r_des = 0;
    double phi_des = 0;

    double phi_diff = 0;

    double x_diff = lm.x - pos.x;
    double y_diff = lm.y - pos.y;

    r_des = sqrt(pow(x_diff,2)+pow(y_diff,2));
    phi_des = atan2(y_diff, x_diff);

    phi_diff = pos.theta + f.phi - phi_des;

    if(f.r-r_des == 0){
       // ROS_INFO_STREAM("pos x: " << pos.x << " y: " << pos.y << " lm x: " << lm.x << " y: " << lm.y);
        //ROS_INFO_STREAM("phidiff: " << phi_diff << " phi_des: " << phi_des << " feature: "<< f.phi << " pos: " << pos.theta);
    }
    while(phi_diff > M_PI_2){
        phi_diff -= 2*M_PI_2;
    }
    while(phi_diff < -M_PI_2){
        phi_diff += 2*M_PI_2;
    }
    if(debug){
        ROS_INFO_STREAM("r-diff: " << f.r - r_des << " gaussR: " << getProbGauss((f.r-r_des), STDDEV_R) << " phi-diff: " << phi_diff << " gaussPhi: " << getProbGauss(phi_diff, STDDEV_PHI) << " weight: " << getProbGauss((f.r-r_des), STDDEV_R)*getProbGauss((phi_diff),STDDEV_PHI));
    }
    return getProbGauss((f.r-r_des), STDDEV_R)*getProbGauss((phi_diff),STDDEV_PHI);//*getProbGauss((f.specifier - lm.specifier), STDDEV_IDF);


}
#endif

/**
 * @brief MeasurementModel::test just for tests
 * @param a
 * @param b
 */
void MeasurementModel::test(double a, double b){
    /* TEST MEASUREMENT */
    pose_t pos;
    pos.x = 1.5*a;
    pos.y = 0.5*b;
    pos.theta = M_PI_2;

    std::vector<feature> f;
    feature f1;
    f1.r = 0.5*b;
    f1.phi = 0;
    f1.knownCorr = -1;
    f1.specifier = GREEN;
    f.push_back(f1);


    /* add error */
    for(int i = 0; i < 100; i++){
        double error = ((double)i-50)/50*0.1*b;
        pos.y = 0.5*b+error;
        ROS_INFO_STREAM("Error: " << error << " Weight: " << get_weight(pos, f));
    }

}


//MeasurementModel::MeasurementModel(const feature &f):feature_(f){}
//LandmarkKnownCorr::LandmarkKnownCorr(const feature &f, const landmark &l):MeasurementModel(f),corrLandmark_(l){}
//LandMarkUnknownCorr::LandMarkUnknownCorr(const feature &f, const std::vector<landmark> &l):MeasurementModel(f),landmarks_(l){}


/*
double LandmarkKnownCorr::get_weight(const pose_t &pos, const feature &f){
    double r_des = 0;
    double phi_des = 0;

    double x_diff = f.x - pos.x;
    double y_diff = f.y - pos.y;

    r_des = sqrt(pow(x_diff,2)+pow(y_diff,2));
    phi_des = atan2(y_diff, x_diff);

    double prob = getProbGauss((f.r-r_des), STDDEV_R)*getProbGauss((f.phi-phi_des),STDDEV_PHI)*getProbGauss((f.specifier - corrL.specifier), STDDEV_IDF);


    return prob;
}
*/



double MeasurementModel::get_weight(const pose_t &pos, const std::vector<feature> &f, bool debug){
    if(f.size()==0){
        //no features detected

        return 0.5; //all particles have the same weight from the measurement model as there is no input
    }

    double weight = 0;
    double feature_count = 0;
    int debugIdx = 0;
    for (auto feature : f){
        feature_count++;
            //unknown correspondence
            double max_weight = 0;
            int debugIdxCurr = 0;
            for (auto landmark : lms_){
                double curr_weight = getCorrProb(pos,feature,landmark);
                if (curr_weight > max_weight){
                    debugIdx = debugIdxCurr;
                    max_weight = curr_weight;
                }
                debugIdxCurr++;
            }
            weight += max_weight*feature.specifier;
            //getCorrProb(pos, feature, lms_[debugIdx], debug);
    }

    //ROS_INFO_STREAM("WEIGHT: " << weight/feature_count);

    return weight;
}
