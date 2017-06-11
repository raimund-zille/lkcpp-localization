#ifndef FILTER_H
#define FILTER_H
#include <vector>
#include <memory>
#include <exception>


class KernelException : public std::exception{
    virtual const char* what() const throw(){
        return "KERNELSIZE EVEN";
    }
};

namespace filter_type {

class Filter {
public:
    Filter(int samples = 3);
    Filter(const Filter &filter);
    virtual std::shared_ptr<std::vector<double>> filter(const std::vector<std::vector<double>> &input) = 0;

    Filter& operator=(const Filter &filter);

protected:
    int samples_;
};

class NoFilter : public Filter {
         /*
          * @brief NoFilter does not manipulate samples
          */
public:
    NoFilter(): Filter() {} //empty constructor

    /*
     * @brief Return input samples
     * @param &input: Reference to samples
     * @return shared_ptr to samples
     */
    std::shared_ptr<std::vector<double>> filter(const std::vector<std::vector<double>> &input) {
                                 std::vector<double> out(input.back());
                                 return std::make_shared<std::vector<double>>(out);
                               }
};


    class Mean : public Filter {
        /*
         * @brief Mean filtering of samples
         */
    public:
        Mean(int samples = 3);
        /*
         * @brief Get mean filtered output of lidar scans
         * @param &input: Reference to samples (since a lidar measurement consists of several samples itself
         *                a multidmensional vector is used as input)
         * @return shared_ptr to mean filtered samples
         */
        std::shared_ptr<std::vector<double>> filter(const std::vector<std::vector<double>> &input);
    };

    class Gauss : public Filter {
        /*
         * @brief Gauss filtering of mean filtered samples
         */
    public:
        /* Gauss Filter Constructor: Make sure that the kernel size is always odd */
        Gauss(double sigma = 0.1, uint kernel_size = 5, int samples = 3);
        /*
         * @brief Get gauss filtered output of mean filtered lidar samples
         * @param &input: Reference to samples (since a lidar measurement consists of several samples itself
         *                a multidmensional vector is used as input)
         * @return shared_ptr to mean and gauss filtered samples
         */
        std::shared_ptr<std::vector<double>> filter(const std::vector<std::vector<double>> &input);

    private:

        double sigma_;
        uint kernel_size_;

    };
}

struct gauss_params{  /// gaussian filter parameters for 1d
    double sigma;     /// sigma
    int kernel_size;  /// kernel size
};

typedef void (*filter)(const std::vector<std::vector<double>> &in, std::vector<double> &out, void *params);

/*
 * @brief get mean values of different lidar samples
 * @param &in: Reference to lidar samples
 * @param &out: Reference to out vector. Store filtered samples in out. Make sure out is empty before it is used
 * @param *params: parameters for filter
 * @return true if mean filtering was successfull, false otherwise
 */
bool mean(const std::vector<std::vector<double>> &in, std::vector<double> &out, void *params = nullptr);

/*
 * @brief get gaussian mean of different lidar samples
 * @param &in: Reference to lidar samples
 * @param &out: Reference to out vector. Store filtered samples in out. Make sure out is empty before it is used
 * @param *params: parameters for filter
 */
void gaussian1D(const std::vector<std::vector<double>> &in, std::vector<double> &out, void *params = nullptr);

#endif
