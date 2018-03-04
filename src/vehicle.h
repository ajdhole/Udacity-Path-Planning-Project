#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

struct SensorFusion {
    int id;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;
};

class Vehicle {
public:
    map<string, int> lane_direction = { {"PLCL", -1},{"LCL", -1},{"LCR", 1},{"PLCR", 1} };
    int preferred_buffer = 30;
    int lane;
    double ref_vel;
    float target_speed;
    int lanes_available;
    float max_acceleration;
    int goal_lane;
    int goal_s;
    string state;

    float v_cost;

    int s_new;
    float v_new;
    float a_new;

    double car_x;
    double car_y;
    double car_s;
    double car_d;
    double car_yaw;
    double car_speed;

    vector<double> prev_path_x;
    vector<double> prev_path_y;
    double end_path_s;
    double end_path_d;

    vector<double> map_wp_x;
    vector<double> map_wp_y;
    vector<double> map_wp_s;
    vector<double> map_wp_dx;
    vector<double> map_wp_dy;

    vector<SensorFusion> sensor_fusion;
    /**
    * Constructor
    */
    Vehicle();
    Vehicle(int lane, float s, float v, float a, string state = "CS", float v_cost = 0.0);

    /**
    * Destructor
    */
    virtual ~Vehicle();

    void configure(double ref_vel, double target_speed, int lanes_available, double goal_s, int goal_lane, double max_acceleration);

    void set_car(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed);

    void set_prev_path(const vector<double> &path_x, const vector<double> &path_y, const double &path_s, const double &path_d);

    void set_map(const vector<double> &wp_x, const vector<double> &wp_y, const vector<double> &wp_s, const vector<double> &wp_dx, const vector<double> &wp_dy);

    void set_sensor_fusion(const vector<SensorFusion> &sensor_fusion);

    void realize_next_state(vector<Vehicle> trajectory);

    void choose_next_state(map<int, vector<Vehicle>> predictions, vector<double>& n_x_vals, vector<double>& n_y_vals);

    vector<string> successor_states();

    void generate_path_points(vector<double>& n_x_vals, vector<double>& n_y_vals);

    vector<Vehicle> generate_predictions(int horizon = 3);

    vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> predictions);

    vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);

    vector<float> get_kinematics(map<int, vector<Vehicle>> predictions, int lane);

    vector<Vehicle> constant_speed_trajectory();

    vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

    vector<Vehicle> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

    void increment(int dt);

    float position_at(int t);

    bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

    bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) const;

};




#endif
