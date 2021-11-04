* Assignment #3 Kalman filter and extended kalman filter:
    - Objectives: This project is divided into two parts:
         + 1) implement kalman filter to estimate the state of an object using LiDAR estimations (see TODOs). The executable generates a file with the trajectory followed by the vehicle (in the file you will see also ground truth and measurement values). Files to modify: kf.cpp, tracking.cpp
                * write a short report explaining the achieved results (run plotter.py to observe the trajectory estimated). Please provide an explanaition of the results
         + 2)  implement an extended kalman filter to estimate the state of an object using lidar and radar measurements. A simulator is provided for the experiments. The executable generates a file with the trajectory followed by the vehicle and RMSE (in the file you will see also the values of the ground truth). Files to modify: tools.cpp (CalculateRMSE), kalman_filter.cpp, fusionEKF.cpp
                * write a short report explaining the achieved results (run plotter.py to observe the trajectory estimated and RMSE). Please provide an explanation of the results
    - Tasks evaluated (15 points):
        + KF update lidar compile and work (5 points)
            + report KF (2 points)
        + EKF Lidar compile and work (6 points)
            + report EKF (2 points)
        
* OS requirements:
    + To run the simulator and the EKF task:
        + Execute all the instructions defined in the file install-ubuntu.sh (in the EKF folder) before compiling the program (this is needed to run the simulator with EKF part)
            ++ libuv1-dev libssl-dev gcc g++ cmake make and uWebSockets are needed so please follow and install are that libraries specified in the install-ubuntu.sh
        + Simulator (use the first scenario)
    
* Instructions to compile the code:
    + KF:
        mkdir build
        cd build
        cmake ..
        make
        ./kf_lidar 
        To generate the plot just write python3 plotter.py (notice that the program only works when the KF have been implemented correctly)

    + EKF:
        mkdir build
        cd build
        cmake ..
        make
        ./ExtendedKF
        As soon as you call ./ExtendedKF executable you can start the simulation 
        To generate the plot just write python3 plotter.py (notice that the program only works when the KF have been implemented correctly)
        
* Important Note:
    + If the simulator does not work for any strange reason, try to change the language of your linux dist. into English.
