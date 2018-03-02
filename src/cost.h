//
// Created by abang on 18-2-27.
//

#ifndef PATH_PLANNING_COST_H
#define PATH_PLANNING_COST_H

#include <math.h>
#include <cmath>
#include "vehicle.h"

class Cost {
public:

    const Vehicle *vehicle;

    vector<vector<double>> sensor_fusion;

    struct PredictState{
        int current_lane = 1;
        int next_lane = 1;

        double target_v = 49.5;
        bool is_collision = false;

        double front_distance = 9999;
        double back_distance = 9999;
        double front_speed = 0;
        double back_speed = 0;

    } predict;

    Cost(const Vehicle *v, vector<vector<double>> s);

    vector<double> CalculateCost(States s);

private:
    /*
     * Cost functions
     */

    const double MS_TO_MPH = 2.23694;

    const double MPH_TO_MS = 0.44704;

    // the weight for different cost function
    const double COLLISION_WEIGHT = 1000;
    const double LANE_CHANGE_WEIGHT = 30;
    const double EFFICIENCY_WEIGHT = 200;

    // use the possiable behavior and predict the state of ego car
    void RealizeState(States s);

    vector<double> GetFrontBackCar(int lane_id);

    double InefficiencyCost();

    double ChangeLaneCost();

    double CollisionCost();

};


#endif //PATH_PLANNING_COST_H
