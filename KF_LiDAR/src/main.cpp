#include <iostream>
#include <sstream>
#include <vector>
#include "Eigen/Dense"
#include "measurement_package.h"
#include "tracking.h"
#include <vector>
#include <fstream>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::ifstream;
using std::istringstream;
using std::string;
using std::vector;
using std::ofstream;

int main() {

  /**
   * Set Measurements
   */
  vector<MeasurementPackage> measurement_pack_list;

  // hardcoded input file with laser and radar measurements
  string in_file_name_ = "../data/obj_pose-laser-radar-synthetic-input.txt";
  ifstream in_file(in_file_name_.c_str(), ifstream::in);

  if (!in_file.is_open()) {
    cout << "Cannot open input file: " << in_file_name_ << endl;
  }

  string line;
  // set i to get only first 3 measurments
  int i = 0;
  while (getline(in_file, line) ) {

    MeasurementPackage meas_package;

    istringstream iss(line);
    string sensor_type;
    iss >> sensor_type; // reads first element from the current line
    int64_t timestamp;
    if (sensor_type.compare("L") == 0) {  // laser measurement
      // read measurements
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      float x;
      float y;
      iss >> x;
      iss >> y;

      meas_package.raw_measurements_ << x,y;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);

    } else if (sensor_type.compare("R") == 0) {
      // Skip Radar measurements
      continue;
    }
    ++i;
  }
  

  // Create a Tracking instance
  Tracking tracking;
    
  // call the ProcessingMeasurement() function for each measurement
  size_t N = measurement_pack_list.size();
  // start filtering from the second frame 
  // (the speed is unknown in the first frame)
  vector<float> x_v;
  vector<float> y_v;
  int count = 0;
  for (size_t k = 0; k < N; ++k) {
    tracking.ProcessMeasurement(measurement_pack_list[k],x_v,y_v);
    cout << count;
    count += 1;
  }
  
  int z=0;

  ofstream  myfile ("../res.txt");
  if (myfile.is_open())
  {
    for(unsigned i = 0; i < x_v.size(); ++i) 
    {
      myfile << x_v[i]<< " "<< y_v[i]<<'\n';

    }
    myfile.close();
  }
  
  if (in_file.is_open()) {
    in_file.close();
  }
  return 0;
}
