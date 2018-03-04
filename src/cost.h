//
//  cost.h
//  Path_Planning
//
//  Created by Anil Dhole on 12/02/18.
//  Copyright © 2018 Anil Dhole. All rights reserved.
//

#ifndef COST_H
#define COST_H
#include "vehicle.h"

using namespace std;

float calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory);

float goal_distance_cost(const Vehicle & vehicle,  const vector<Vehicle> & trajectory,  const map<int, vector<Vehicle>> & predictions, map<string, float> & data);

float inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data);

float lane_speed(const map<int, vector<Vehicle>> & predictions, int lane);

map<string, float> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions);


#endif /* cost_h */
