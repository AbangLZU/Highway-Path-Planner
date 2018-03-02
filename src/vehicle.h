//
// Created by abang on 18-2-27.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <iostream>
#include <vector>

using namespace std;
enum States {KL, LCL, LCR};



class Vehicle {

public:
    States state = KL;

    int lane = 0;
    double ref_vel = 0;

    double s = 0;
    double d = 0;
    double speed = 0;

    double delta_t = 0;



    //the constructor
    Vehicle(int lane, double ref_vel);

    //update the status of the car
    void UpdateStates(double s, double d, double speed, double delta_t);

    void ChooseNextState(vector<vector<double>> sensor_fusion);

private:
    vector<States> GetReachableState();

    void RealizeNextState(States next_state, vector<vector<double>> sensor_fusion);

};


#endif //PATH_PLANNING_VEHICLE_H
