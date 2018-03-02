//
// Created by abang on 18-2-27.
//

#include "vehicle.h"
#include "cost.h"

Vehicle::Vehicle(int lane, double ref_vel) {
    this->lane = lane;
    this->ref_vel = ref_vel;
}

void Vehicle::UpdateStates(double s, double d, double speed, double delta_t) {

    this->s = s;
    this->d = d;

    this->speed = speed;
    this->delta_t = delta_t;
}



void Vehicle::ChooseNextState(vector<vector<double>> sensor_fusion) {

    vector <States> reachable_states = GetReachableState();

    double min_cost = 9999;
    States best_state = KL;
    double final_target_speed = 49.5;
    int final_ref_lane = this->lane;

    //calculate the cost function for all reachable state
    for (int i = 0; i < reachable_states.size(); ++i) {
        auto sts = reachable_states[i];
        auto cost_function = Cost(this, sensor_fusion);
        auto result_vector = cost_function.CalculateCost(sts);

        double cost_value = result_vector[0];
        double target_speed = result_vector[1];

        if (cost_value < min_cost){
            min_cost = cost_value;

            final_target_speed = target_speed;
            best_state = sts;
            final_ref_lane = cost_function.predict.next_lane;
        }
    }

    if (this->ref_vel < final_target_speed && this->ref_vel < 49.5) {
        this->ref_vel += 0.224;
    } else if (this->ref_vel > final_target_speed && this->ref_vel > 0) {
        this->ref_vel -= 0.224;
    }

    this->lane = final_ref_lane;

    cout<<"The best state is: "<<best_state<<" with cost: "<<min_cost<<" target lane: "<<final_ref_lane<<" target vel: "<<final_target_speed<<endl;
}

vector<States> Vehicle::GetReachableState() {
    // all possiable states
    vector<States> r_state;

    if (this->state == KL){
        r_state.push_back(KL);
        if (this->lane != 0){
            // check the lane again
            if (d < (2 + 4 * (lane) + 2) && d > (2 + 4 * (lane) - 2) && speed > 20){
                r_state.push_back(LCL);
            }
        }
        if (this->lane != 2){
            // check the lane again
            if (d < (2 + 4 * (lane) + 2) && d > (2 + 4 * (lane) - 2) && speed > 20){
                r_state.push_back(LCR);;
            }
        }
    } else if (this->state == LCL || this->state == LCR){
        r_state.push_back(KL);
    }
    return r_state;
}

void Vehicle::RealizeNextState(States next_state, vector<vector<double>> sensor_fusion) {

}



