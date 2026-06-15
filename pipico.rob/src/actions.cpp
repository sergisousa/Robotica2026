/* Copyright (c) 2025  Paulo Costa
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */

#include "actions.h"
#include "htransf_2d.h"
#include "graph.h"

action_t action;


float sqr(float x)
{
  return x * x;
}

float sign(float x)
{
  if (x > 0) return 1;
  else if (x < 0) return -1;
  else return 0;
}

float norm(float x, float y)
{
  return sqrt(x * x + y * y);
}

void normalize(float& x, float& y)
{
  float norm = sqrt(x * x + y * y);
  if (norm == 0) return;
  x = x / norm;
  y = y / norm;
}


float dist(float x0, float y0, float x1, float y1)
{
  return sqrt(sqr(x1 - x0) + sqr(y1 - y0));
}

// Normalize angle to the range of [-π, π]
float normalize_angle(float angle)
{
  if (fabs(angle) < M_PI)
    return angle;

  if (angle >= 0) {
    angle = fmod(angle + PI, TWO_PI);
    return angle - PI;
  } else {
    angle = fmod(-angle + PI, TWO_PI);
    return -(angle - PI);
  }
}

float dif_angle(float a0, float a1)
{
  return normalize_angle(normalize_angle(a0) - normalize_angle(a1));
}

float top_hat_squared(float w, float wz)
{
   float v = -(w - wz) * (w + wz) / sqr(wz);
   if (v < 0) v =  0;

   return v;
}


action_t::action_t()
{
  Pf.x = 1;
  Pf.y = 0;
  thetaf = PI/2;
  thetai = PI/2;

  e_theta_tresh = radians(2);
  e_xy_tresh = 0.02;

  C.x = 0;
  C.y = 0.5;
  Pi.x = 0;
  Pi.y = 0;
  v_nom = 0.15;
  radius = C.distance(Pi);//  dist(C.x, C.y, xi, yi);
  alpha = 4;

  ktrack = -0.1;
  wz = 4;
  w0 = 4;

  kset_theta = 3.0;
  ktheta = 5.0;
  kn = -50;

  stop_at_end = false;
}

void action_t::stop(void)
{
  robot.v_req = 0;
  robot.w_req = 0;
}

void action_t::set_theta(void)
{
  robot.v_req = 0;
  e_theta = dif_angle(thetaf, robot.thetae);
  if (fabs(e_theta) < e_theta_tresh) {
    done = true;
    robot.w_req = 0;
    return;
  }

  robot.w_req = kset_theta * e_theta;
}

void action_t::goto_xy(void)
{
  e_xy = dist(Pf.x, Pf.y, robot.xe, robot.ye);

  thetaf = atan2(Pf.y - robot.ye, Pf.x - robot.xe);
  e_theta = dif_angle(thetaf, robot.thetae);

  robot.v_req = v_nom * top_hat_squared(e_theta, M_PI/2);
  robot.w_req = ktheta * e_theta;

  next_step = false;

  if (e_xy < e_xy_tresh)
  {
    done = true;
    robot.v_req = 0;
    robot.w_req = 0;
  }
}


void action_t::follow_line(void)
{
  e_xy = dist(Pf.x, Pf.y, robot.xe, robot.ye);

  // theta error
  thetaf = atan2(Pf.y - robot.ye, Pf.x - robot.xe);
  e_theta = dif_angle(thetaf, robot.thetae);

  // distance error
  float uifx = Pf.x - Pi.x;
  float uify = Pf.y - Pi.y;

  float vifx = uifx/sqrt(sqr(uifx) + sqr(uify));
  float vify = uify/sqrt(sqr(uifx) + sqr(uify));

  float uirx = robot.xe - Pi.x;
  float uiry = robot.ye - Pi.y;

  float e_n = vifx * uiry - vify * uirx;

  // finally
  robot.w_req = ktheta * e_theta + kn * e_n;
  robot.v_req = v_nom * top_hat_squared(robot.w_req, w0);
  
  if (e_xy < e_xy_tresh)
  {
    done = true;
    robot.v_req = 0;
    robot.w_req = 0;
  }
}


void action_t::follow_circle(void)
{
  // e_xy = dist(Pf.x, Pf.y, robot.xe, robot.ye);

  // distance error
  float r = dist(Pi.x, Pi.y, C.x, C.y);
  float d = dist(robot.xe, robot.ye, C.x,  C.y);

  float e_n  = r - d;

  //theta error
  float uCrx = robot.xe - C.x;
  float uCry = robot.ye - C.y;

  float beta = sign(alpha) * M_PI_2;

  float vtx = cos(beta) * uCrx - sin(beta)* uCry;
  float vty = sin(beta) * uCrx + cos(beta)* uCry;

  float thetat = atan2(vty, vtx);
  float e_theta = dif_angle(thetat, robot.thetae);
  
  robot.w_req = v_nom/r + ktheta * e_theta + kn * e_n;
  robot.v_req = v_nom * top_hat_squared(robot.w_req - v_nom/r, w0);

  //obtain final position
  float uCix = Pi.x - C.x;
  float uCiy = Pi.y - C.y;
  float theta_Ci = atan2(uCiy, uCix);

  float uCfx = r*cos(theta_Ci + alpha);
  float uCfy = r*sin(theta_Ci + alpha);

  Pf.x = C.x + uCfx;
  Pf.y = C.y + uCfy;

  //stop condition
  // float uCfx = Pf.x - C.x;
  // float uCfy = Pf.y - C.y;

  // float err_rot = sign(alpha) * sign(uCfx * uCry - uCfy * uCrx);

  float err_2 = acos((uCfx * uCrx + uCfy * uCry)/(r * sqrt(sqr(uCrx) + sqr(uCry))));

  if (abs(err_2) < e_theta_tresh){
    done = true;
    robot.v_req = 0;
    robot.w_req = 0;
  }
}


void action_t::follow_track(void)
{
  // Important variables

  // robot.IRLine.found_center: boolean to indicate that the lline sensor has seen the track
  // if it is false the robot sensors are only seing white

  // robot.IRLine.pos_center: the estimated distance (in mm) from the center of the track to the center of the sensor
  // is is only valid if robot.IRLine.found_center is true

  // ktrack: a gain that you can use to calculate the desired angular speed
  // w0, wz: some constants that you can use in your calculations

  // v_nom: the desired linear speed

  // You must set the values of:
  // robot.w_req: the desired angular speed in rad/s
  // robot.v_req: the desired linear speed in m/s (max value ~ 0.4 m/s)

  if (robot.IRLine.found_center) {
    robot.w_req = ktrack * robot.IRLine.pos_center;
    //robot.v_req = v_nom;
    //robot.v_req = -(v_nom / sqr(wz)) * (robot.w_req - wz) * (robot.w_req + wz);
    //if (robot.v_req < 0) robot.v_req =  0;
    robot.v_req = v_nom * top_hat_squared(robot.w_req, wz);

  } else {
    // Stop and rotate to find the line again
    robot.w_req = w0 * sign(robot.w_req);
    robot.v_req = 0;
  }

}


void action_t::follow_track_digital(void)
{
  if (robot.IRLine.found_center_digital) {
    robot.w_req = ktrack * robot.IRLine.pos_center_digital;
    robot.v_req = v_nom * top_hat_squared(robot.w_req, wz);

  } else {
    // Stop and rotate to find the line again
    robot.w_req = w0 * sign(robot.w_req);
    robot.v_req = 0;
  }

}


void action_t::follow_track_right(void)
{
  robot.w_req = ktrack * robot.IRLine.pos_right;
  //w_req = w_req * fabs(w_req);
  //v_req = fmax(0, Vnom - 0.1 * fabs(w_req));
  robot.v_req = v_nom;
}


void action_t::follow_track_left(void)
{
  robot.w_req = ktrack * robot.IRLine.pos_left;
  //w_req = w_req * fabs(w_req);
  //v_req = fmax(0, Vnom - 0.1 * fabs(w_req));
  robot.v_req = v_nom;
}

void action_t::follow_wall_right(void)
{
  robot.w_req = ktrack * (wall_dist_right_ref - robot.tof_dist_right);
  robot.v_req = v_nom * top_hat_squared(robot.w_req, wz);
}


void action_t::follow_wall_left(void)
{
  robot.w_req = 0;
  robot.v_req = v_nom;
}

void action_t::robot_at_factory(void)
{
  robotatfactory = 1;
  next_step = false;
  if (!traj_done) {
    done = false;
    idx_path = 0;
    float current_pos[2] = {robot.xe, robot.ye};
    int start_node = find_nearest_node(current_pos);
    a_star(start_node, find_node_idx_by_label(goal_node), path); // 19 is the node for the tests -> Index 19 : Aruco ID 26
    traj_done = 1;
  } else if (path[idx_path] != -1) {
    Pf.x = node_coords[path[idx_path]][0];
    Pf.y = node_coords[path[idx_path]][1];
    idx_path += 1;
    next_step = true;
  } else {
    robotatfactory = 0;
    traj_done = 0;
    done = true;
  }
}


void action_t::do_action_list(void)
{
  robot.w_req = 0;
  robot.v_req = 0;
}

void action_t::do_action(void)
{

}


//-------------------------------------------------------
// segment_t


segment_t::segment_t()
{
  Pi.set(0, 0);
  Pf.set(1, 0);
  refresh();
}

segment_t::segment_t(Vec2f thePi, Vec2f thePf)
{
  Pi = thePi;
  Pf = thePf;
  refresh();
}

void segment_t::refresh()
{
  angle = atan2(Pf.y - Pi.y, Pf.x - Pi.x);
  dist = Pi.distance(Pf);
}


void segment_t::set_points(float xi, float yi, float xf, float yf)
{
  Pi.set(xi, yi);
  Pf.set(xf, yf);
  refresh();
}


