#ifndef TRACKING_H_
#define TRACKING_H_

#include <vector>
#include <string>
#include <fstream>
#include "kf.h"
#include "measurement_package.h"
using std::vector;
class Tracking {
 public:
  Tracking();
  virtual ~Tracking();
  void ProcessMeasurement(const MeasurementPackage &measurement_pack, std::vector<float> &x_v,std::vector<float> &y_v);

  KalmanFilter kf_;

 private:
  bool is_initialized_;
  int64_t previous_timestamp_;

  //acceleration noise components
  float noise_ax;
  float noise_ay;
  float noise_ax_2;
  float noise_ay_2;

};

#endif  //TRACKING_H_
