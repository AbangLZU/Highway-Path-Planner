//
// Created by abang on 18-2-27.
//

#include "cost.h"

Cost::Cost(const Vehicle *v, vector<vector<double>> s) {
    this->vehicle = v;
    this->sensor_fusion = s;
}

/**
 *
 * @param s
 * @return a vector that contain the cost and target speed of States s
 */
vector<double> Cost::CalculateCost(States s) {

    RealizeState(s);

    double cost = 0;
    cost += InefficiencyCost();
    cost += ChangeLaneCost();
    cost += CollisionCost();

    double result_velocity = predict.target_v;

    vector<double> result = {cost, result_velocity};
    return result;
}

void Cost::RealizeState(States s) {
    if (s == KL){
        // the lane will not change
        predict.current_lane = vehicle->lane;
        predict.next_lane = predict.current_lane;
    } else if (s == LCL){
        predict.current_lane = vehicle->lane;
        predict.next_lane = predict.current_lane - 1;
    } else if (s == LCR){
        predict.current_lane = vehicle->lane;
        predict.next_lane = predict.current_lane + 1;
    }

    // check if the current and next is legal
    if (predict.next_lane < 0){
        predict.next_lane = 0;
    }
    if (predict.next_lane > 2){
        predict.next_lane = 2;
    }

    auto next_lane_car = GetFrontBackCar(predict.next_lane);

    predict.front_distance = next_lane_car[0];
    predict.back_distance = next_lane_car[2];
    predict.front_speed = next_lane_car[1];
    predict.back_speed = next_lane_car[3];


    if (predict.back_distance < 10) {
        // the next state front car is too close
        predict.is_collision = true;
        predict.target_v = predict.back_speed + 2;

    } else if (predict.front_distance < 30){
        // the next state front car is too close
        predict.is_collision = true;
        predict.target_v = predict.front_speed;
    }
}


/**
 *
 * @param lane_id
 * @return a vector of double: front_distance, front_velocity, back_distance, back_velocity
 * @brief calculate the front car distance (s axis) and speed in lane lane_id
 */
vector<double> Cost::GetFrontBackCar(int lane_id) {
    vector<double> distance_vel;

    double front_speed = 49.5;
    double front_distance = 9999;
    double back_speed = 49.5;
    double back_distance = 9999;

    for (int i = 0; i < sensor_fusion.size(); ++i) {
        double obj_d = sensor_fusion[i][6];
        // the object within current lane
        if ((obj_d < (2 + 4 * lane_id + 2)) && (obj_d > (2 + 4 * lane_id - 2))){
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];

            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = sensor_fusion[i][5];
            check_car_s += ((double)vehicle->delta_t * check_speed);

            double distance_to_it = check_car_s - vehicle->s;

            // the car front of us
            if (check_car_s > vehicle->s){
                // the car in current lane is too close
                if (front_distance > distance_to_it) {
                    front_speed = check_speed * MS_TO_MPH;
                    front_distance = distance_to_it;
                }
            }
            // the back car
            if (check_car_s < vehicle->s){
                distance_to_it = abs(distance_to_it);
                if (back_distance > distance_to_it) {
                    back_speed = check_speed * MS_TO_MPH;
                    back_distance = distance_to_it;
                }
            }
        }
    }

    distance_vel.push_back(front_distance);
    distance_vel.push_back(front_speed);
    distance_vel.push_back(back_distance);
    distance_vel.push_back(back_speed);

    return distance_vel;
}

/**
 *
 * @return the change lane cost
 * the vehicle should not always change lane, if the speed is limit speed and no danger, the vehicle should keep lane
 */
double Cost::ChangeLaneCost() {
    double cost = 0;
    if (predict.current_lane != predict.next_lane){
        cost += LANE_CHANGE_WEIGHT;
    }
    return cost;
}

/**
 *
 * @return the inefficiency cost
 * Always, the best efficiency is when the target speed is closest to the limit
 */
double Cost::InefficiencyCost() {
    double cost = 0;
    double diff = (49.5 - predict.target_v)/49.5;
    cost = abs(diff) * EFFICIENCY_WEIGHT;
    return cost;
}

/**
 *
 * @return The collision cost
 *
 */

double Cost::CollisionCost() {
    double cost = 0;
    if (predict.is_collision){
        double front_collide_time = 9999;
        double back_collide_time = 9999;

        if (predict.front_distance != 9999){
            //distance divided by the relative speed
            double relative_speed = vehicle->speed - predict.front_speed;
            if (relative_speed > 0){
                // we are now faster than the front car and the distance is < 30 m
                front_collide_time = predict.front_distance / (relative_speed * MPH_TO_MS);
            }
        }
        if (predict.back_distance != 9999){
            double relative_speed = predict.back_speed - vehicle->speed;
            if (relative_speed > 0){
                // we are now slower than the back car and the distance is < 30 m
                back_collide_time = predict.back_distance / (relative_speed * MPH_TO_MS);
            }
        }

        if (front_collide_time <= back_collide_time){
            cost = exp(-pow(front_collide_time, 2)) * COLLISION_WEIGHT;
        } else if(front_collide_time > back_collide_time){
            cost = exp(-pow(back_collide_time, 2)) * COLLISION_WEIGHT;
        }

        if ((predict.next_lane != predict.current_lane) && (front_collide_time > 1.5) && (back_collide_time > 1.5)){
            cost = cost/10.0;
        }

        if((front_collide_time == 9999) && (back_collide_time == 9999)){
            cost = 0;
        }
    }
    return cost;
}

