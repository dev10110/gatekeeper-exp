#pragma once

#include <cmath>
#include <iostream>
#include <vector>

namespace dyn {

typedef struct State {
  double x;
  double y;
  double z;
  double vx;
  double vy;
  double vz;
  double yaw;
} State;

typedef struct Input {
  double ax;
  double ay;
  double az;
  double yaw_rate;
} Input;

typedef struct Trajectory {
  std::vector<double> ts;
  std::vector<State> xs;
  std::vector<Input> us;
} Trajectory;

State copy(State x);

Input copy(Input u);

State dynamics(State &x, Input &u, double dt);

Input tracking_controller(State &x, State &x_des);

Trajectory simulate_hover(double t0, State &x0, State &x_des, float dt,
                          size_t N);

Trajectory simulate_extend_hover(double t0, State &x0, Trajectory &P_nom,
                                 size_t N_nom, size_t N_ext, double dt);

double clampToPi(double x);
double angle_diff(double x, double y); // returns x-y but with care;
double interpolate_angles(double x, double y, double f);

} // namespace dyn
