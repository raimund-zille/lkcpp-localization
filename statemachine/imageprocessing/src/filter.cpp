#include <filter.h>
#include <iterator>
#include <math.h>
#include <iostream>
#include <ros/ros.h>
using namespace std;
using namespace filter_type;
typedef std::vector<double>::iterator iter;

/* 1d convolution for lidar samples */
static bool convolution1D(const vector <double> &kernel, const vector<double> &in, vector<double> &out){

    /* return if size is not matching */
    if (!out.empty()) return false;

    /* make sure kernel size is odd, else the shit is burning =D */
    if(kernel.size() % 2 == 0) return false;
    int k_size = kernel.size();
    int s_size = in.size();

    //ROS_INFO_STREAM("SIZES: " << k_size << " " << in.size());
    for (int i = 0; i < k_size + s_size -1; i++){

        double sum = 0;
        int const j_min = (i >= s_size - 1)? i - (s_size - 1) : 0;
        int const j_max = (i <  k_size - 1)? i : k_size - 1;


        for (int j =  j_min; j <= j_max; j++){
            sum += in[i - j] * kernel[j];
        }
        out.push_back(sum);
        //ROS_INFO_STREAM("SUM " << sum);
    }

    /* remove overhead */

    int overhead = out.size()-in.size();
    int cut_iter = overhead / 2;

    /* remove 2 elements per iteration */
    while(cut_iter-= 2 > 0 ){
        out.erase(out.begin());
        out.pop_back();
    }

}

bool mean(const vector<vector<double>> &in, vector<double> &out, void *params){

    (void*)(params);

    /* return if size is not matching */
    if (out.size() != in.back().size()) return false;

    /* iterate through vector in */
    int samples = 0;
    for (auto vec : in){
        for(int i = 0; i < in.size(); i++) out[i] += vec[i];
        samples++;
    }

    /* if there are no samples return */
    if (samples == 0) return false;

    for (auto val : out) val /= samples;
    return true;
}

void gaussian1D(const vector<vector<double>> &in, vector<double> &out, void *params){

    /* if params ar not set return */
    if (params == nullptr) return;

    /* return if out is not empty */
    if (!out.empty()) return;

    /* first get mean of all samples */
    gauss_params *p = static_cast<gauss_params*>(params);
    mean(in,out,params);

    double denominator =  p->sigma * sqrt(2*M_PI);

    auto gauss_sample = [denominator](double val, double sigma){
        return (1./denominator)*exp(-(val*val)/(denominator*denominator));
    };

    vector<double> kernel;

    /* create kernel */
    for(int i = -(p->kernel_size/2); i <= p->kernel_size/2; i++){
        kernel.push_back(gauss_sample(i,p->sigma));
    }

    /* 1d convolution */
    vector<double> current_out(out);
    convolution1D(kernel,current_out,out);

}

Mean::Mean(int samples):Filter(samples){}

shared_ptr<vector<double>> Mean::filter(const vector<vector<double>> &input){
    int sample = input.size();
    vector<double>out;

    /* setup start index */
    int start_index = input.size() < this->samples_ ? 0 : input.size() - this->samples_;

    /* iterate through all columns*/
    for (int i =  start_index; i < input.back().size(); i++){
        double sum = 0;

        /* iterate through all rows */
        for (int j = 0; j < input.size(); j++) sum += input[j][i];

        out.push_back(sum/sample);
    }

    return make_shared<vector<double>>(out);
}

Gauss::Gauss(double sigma, uint kernel_size, int samples):Filter(samples), sigma_(sigma)
{
    KernelException except;

    try{
        bool is_odd = (kernel_size % 2) == 0;
        if (is_odd) throw except;
        else kernel_size_ = kernel_size;
    }catch(exception &e){
        std::cout << e.what() << endl;
        kernel_size_ = 5;
    }

}


shared_ptr<vector<double>> Gauss::filter(const vector<vector<double>> & input){


     double denominator =  sigma_ * sqrt(2*M_PI);

     auto gauss_sample = [denominator](double val, double sigma){
        return (1./denominator)*exp(-(val*val)/(denominator*denominator));
     };

     vector<double> kernel;

     /* create kernel */
     for(int i = -(kernel_size_/2); i != (kernel_size_/2 + 1); i++){
         kernel.push_back(gauss_sample(-i,sigma_));
     }

     vector<double> output;

     /* setup start index */
     int start_index = input.size() < this->samples_ ? 0 : input.size() - this->samples_;

     /* iterate through all columns */
     for (int i = start_index; i < input.back().size(); i++){

         vector<double> current;
         vector<double> convol;

         /* extract all rows of current columns*/
         for(int j = input.size(); j < input.size(); j++) current.push_back((input[j][i]));

         /* convolute current samples and append result to output vector */
         convolution1D(kernel,current,convol);

         /* average the samples */
         int samples = convol.size();
         double sum = 0;
         for (auto element : convol) sum+=element;
         sum /= samples;

         /* append to output vector*/
         output.push_back(sum);
     }

     return make_shared<vector<double>>(output);

}

Filter::Filter(int samples):samples_(samples){}
Filter::Filter(const Filter &filter){
    this->samples_ = filter.samples_;
}

Filter &Filter::operator =(const Filter &filter){
    this->samples_ = filter.samples_;
}
