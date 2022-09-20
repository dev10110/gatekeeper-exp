
#include "gatekeeper/dynamics.hpp"

namespace dyn {

size_t size(Trajectory &P) { return P.ts.size(); }

void print(State &x) {
  std::cout << "  " << x.x << ", " << x.y << ", " << x.z << std::endl;
}

void print(Trajectory &P) {
  std::cout << "Trajectory: " << std::endl;
  for (size_t i = 0; i < size(P); i++) {
    print(P.xs[i]);
  }
}

State copy(State x) {
  State xc{};
  xc.x = x.x;
  xc.y = x.y;
  xc.z = x.z;
  xc.vx = x.vx;
  xc.vy = x.vy;
  xc.vz = x.vz;
  xc.yaw = x.yaw;
  return xc;
}

Input copy(Input u) {
  Input uc{u.ax, u.ay, u.az, u.yaw_rate};
  return uc;
}

State dynamics(State &x, Input &u, double dt) {

  State xp{};

  xp.x = x.x + x.vx * dt + 0.5 * u.ax * dt * dt;
  xp.y = x.y + x.vy * dt + 0.5 * u.ay * dt * dt;
  xp.z = x.z + x.vz * dt + 0.5 * u.az * dt * dt;

  xp.vx = x.vx + u.ax * dt;
  xp.vy = x.vy + u.ay * dt;
  xp.vz = x.vz + u.az * dt;

  xp.yaw = clampToPi(x.yaw + u.yaw_rate * dt);

  return xp;
}

Input tracking_controller(State &x, State &x_des) {

  Input u{};

  double kp = 4.0;
  double kv = 2.0;

  u.ax = kp * (x_des.x - x.x) + kv * (x_des.vx - x.vx);
  u.ay = kp * (x_des.y - x.y) + kv * (x_des.vy - x.vy);
  u.az = kp * (x_des.z - x.z) + kv * (x_des.vz - x.vz);

  double angle_diff = clampToPi(x_des.yaw - x.yaw);
  u.yaw_rate = 1.0 * std::min(M_PI / 4.0, std::max(angle_diff, -M_PI / 4.0));

  return u;
}

Trajectory simulate_hover(double t0, State &x0, State &x_des, float dt,
                          size_t N) {

  Trajectory P{};

  // make a copy
  State x(x0);
  double t = t0;

  for (size_t i = 0; i < N; i++) {

    Input u = tracking_controller(x, x_des);

    P.ts.push_back(t);
    P.xs.push_back(copy(x));
    P.us.push_back(copy(u));

    x = dynamics(x, u, dt);
    t = t + dt;
  }

  return P;
}

double clampToPi(double d) { return std::atan2(std::sin(d), std::cos(d)); }

// double clampTo2Pi(double d){
//   d = clampToPi(d);
//   return ( d >= 0 ) ? d : (2*M_PI + d);
// }
//
// double angle_diff(double x, double y){ // returns x-y
//
//   x = clampTo2Pi(x);
//   y = clampTo2Pi(y);
//
//   if (x < (y + M_PI)) {
//     return x - y;
//   }
//

//  return std::atan2(std::sin(x-y), std::cos(x-y));
//}

double interpolate_angles(double x, double y, double f) {
  double c = (1 - f) * std::cos(x) + f * std::cos(y);
  double s = (1 - f) * std::sin(x) + f * std::sin(y);
  return std::atan2(s, c);
}

Trajectory interpolate(Trajectory &P_des, float dt) {

  // std::cout << "** INTERPOLATING: " << std::endl;
  // print(P_des);

  Trajectory P{};

  double t = P_des.ts[0];
  size_t ind = 0;
  double Delta = P_des.ts[1] - P_des.ts[0];

  // std::cout << "Delta: " << Delta << std::endl;

  while (ind < P_des.ts.size() - 1) {
    while (t >= P_des.ts[ind + 1]) {
      ind = ind + 1;
    }
    double f = (t - P_des.ts[ind]) / Delta;

    // std::cout << "f" << std::endl;

    State x = {f * P_des.xs[ind + 1].x + (1.0 - f) * P_des.xs[ind].x,
               f * P_des.xs[ind + 1].y + (1.0 - f) * P_des.xs[ind].y,
               f * P_des.xs[ind + 1].z + (1.0 - f) * P_des.xs[ind].z,
               f * P_des.xs[ind + 1].vx + (1.0 - f) * P_des.xs[ind].vx,
               f * P_des.xs[ind + 1].vy + (1.0 - f) * P_des.xs[ind].vy,
               f * P_des.xs[ind + 1].vz + (1.0 - f) * P_des.xs[ind].vz,
               interpolate_angles(P_des.xs[ind].yaw, P_des.xs[ind + 1].yaw, f)};
    // f * P_des.xs[ind + 1].yaw + (1.0 - f) * P_des.xs[ind].yaw}; // do a
    // correct interpolation

    Input u = {f * P_des.us[ind + 1].ax + (1.0 - f) * P_des.us[ind].ax,
               f * P_des.us[ind + 1].ay + (1.0 - f) * P_des.us[ind].ay,
               f * P_des.us[ind + 1].az + (1.0 - f) * P_des.us[ind].az,
               f * P_des.us[ind + 1].yaw_rate +
                   (1.0 - f) * P_des.us[ind].yaw_rate};

    P.ts.push_back(t);
    P.xs.push_back(x);
    P.us.push_back(u);
    t += dt;
  }

  // std::cout << "** INTERPOLATED: " << std::endl;
  // print(P);

  return P;
}

Trajectory simulate_extend_hover(double t0, State &x0, Trajectory &P_nom,
                                 size_t N_nom, size_t N_ext, double dt) {

  Trajectory P_des = interpolate(P_nom, dt);

  Trajectory P{};

  // make a copy
  State x(x0);
  double t = t0;

  double des_hover = x.yaw;

  for (size_t i = 0; i < N_nom; i++) {

    if (i >= P_des.xs.size())
      break;

    Input u = tracking_controller(x, P_des.xs[i]);

    P.ts.push_back(t);
    P.xs.push_back(copy(x));
    P.us.push_back(copy(u));

    des_hover = P_des.xs[i].yaw;

    x = dynamics(x, u, dt);
    t = t + dt;
  }

  // define the desired hover state
  State hover = copy(x);
  hover.vx = 0.0;
  hover.vy = 0.0;
  hover.vz = 0.0;
  hover.yaw = des_hover;

  for (size_t i = 0; i < N_ext; i++) {

    Input u = tracking_controller(x, hover);

    P.ts.push_back(t);
    P.xs.push_back(copy(x));
    P.us.push_back(copy(u));

    x = dynamics(x, u, dt);
    t = t + dt;
  }

  // std::cout << "** SIMED: " << std::endl;
  // print(P);

  return P;
}

} // namespace dyn
