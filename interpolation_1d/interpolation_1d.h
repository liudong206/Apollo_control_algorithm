#ifndef INTERPOLATION_1D_H
#define INTERPOLATION_1D_H
#include <memory>
#include <utility>
#include <vector>
#include <map>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/unsupported/Eigen/Splines"

namespace control {

class Interpolation1D{
public:
    typedef std::vector<std::pair<double,double>> DataType;
    Interpolation1D() = default;

    //return true if init is ok
    bool Init(const DataType& xy);

    //only initerplation x between [x_min,x_max]
    //for x out of range,start or end y value is returned.
    double Interpolate(double x) const;

private:
    //helper to scale x values down to [0,1]
    double ScaledValue(double x) const;

    Eigen::RowVectorXd ScaledValues(Eigen::VectorXd const& x_vec) const;

    double x_min_ = 0.0;
    double x_max_ = 0.0;
    double y_start_ = 0.0;
    double y_end_ = 0.0;

    //spline of one-dimensional "points"
    std::unique_ptr<Eigen::Spline<double,1>>spline_;
};
//map{speed ratio}
extern std::map<double,double> lat_err_gain_scheduler;
extern std::map<double,double> heading_err_gain_scheduler;
} //namespace
#endif // INTERPOLATION_1D_H
