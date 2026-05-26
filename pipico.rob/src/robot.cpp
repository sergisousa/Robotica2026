/* Copyright (c) 2021  Paulo Costa
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

#include <Arduino.h>
#include <math.h>
#include "robot.h"
#include "actions.h"

robot_t robot;

template <typename T> int sign(T val)
{
  return (T(0) < val) - (val < T(0));
}

robot_t::robot_t()
{
  stoped = false;
  wheel_dist = 0.132;  //0.120;
  wheel_radius = 0.056 / 2;  //0.0689 / 2;
  gear_ratio = 120;
  encoder_poles = 8;

  dv_max = 5;
  dw_max = 10;
  dt = 0.04;

  follow_k = -0.15;
  follow_v = 0.20;

  i_lambda = 0;
  led = 0;

  lidar_mot_u = 3.3;

  no_line_interval = 0.5;
  mean_abs_w_tresh = 0.5;
  align_angle_tresh = radians(20);

  pchannels = NULL;
  pfsm = NULL;
}

void robot_t::odometry(void)
{
  // Estimate wheels speed using the encoders
  // motor gear redution = 1:120
  // Encoder pulses = 8
  // odometry conts per pulse = 4

  w1e = enc1 * (TWO_PI / (4.0 * encoder_poles * gear_ratio));
  w2e = enc2 * (TWO_PI / (4.0 * encoder_poles * gear_ratio));

  v1e = w1e * wheel_radius;
  v2e = w2e * wheel_radius;

  // Estimate robot speed
  ve = (v1e + v2e) / 2.0;
  we = (v1e - v2e) / wheel_dist;

  // Estimate the distance and the turn angle
  ds = ve * dt;
  dtheta = we * dt;

  // Save previous value
  prev_xe = xe;
  prev_ye = ye;
  prev_thetae = thetae;

  // Estimate pose
  xe += ds * cos(thetae + dtheta/2);
  ye += ds * sin(thetae + dtheta/2);
  thetae = normalize_angle(thetae + dtheta);

  // Relative displacement
  rel_s += ds;
  rel_theta += dtheta;
}

void robot_t::update_aruco(float x, float y, float theta, int count)
{
  const float factor = 0.1;

  xe = xe + (x - xe) * factor;
  ye = ye + (y - ye) * factor;

  float delta = dif_angle(theta, thetae);
  thetae = normalize_angle(thetae + delta * factor);
}

void robot_t::setRobotVW(float Vnom, float Wnom)
{
  v_req = Vnom;
  w_req = Wnom;
}


void robot_t::accelerationLimit(void)
{
  float dv = v_req - vref;
  dv = constrain(dv, -dv_max, dv_max);
  vref += dv;

  float dw = w_req - wref;
  dw = constrain(dw, -dw_max, dw_max);
  wref += dw;
}


void robot_t::calcMotorsVoltage(void)
{
  if (control_mode == cm_voltage) {
    u1 = u1_req + sign(u1_req) *  PID[0].ppars->dead_zone;
    u2 = u2_req + sign(u2_req) *  PID[1].ppars->dead_zone;
    return;

  } else if (control_mode == cm_pid) {
    w1ref = w1_req;
    w2ref = w2_req;

  } else if (control_mode == cm_kinematics) {
    v1ref = vref + wref * wheel_dist / 2;
    v2ref = vref - wref * wheel_dist / 2;

    w1ref = v1ref / wheel_radius;
    w2ref = v2ref / wheel_radius;
  }


  if (w1ref != 0)
    u1 = PID[0].calc(w1ref, w1e) + sign(w1ref) * PID[0].ppars->dead_zone;
  else {
    u1 = 0;
    PID[0].Se = 0;
    PID[0].y_ref = 0;
  }

  if (w2ref != 0)
    u2 = PID[1].calc(w2ref, w2e) + sign(w2ref) * PID[1].ppars->dead_zone;
  else {
    u2 = 0;
    PID[1].Se = 0;
    PID[1].y_ref = 0;
  }
}



void robot_t::send_command(const char* command, float par)
{
  if (pchannels) pchannels->send_command(command, par);
}

void robot_t::send_command(const char* command, const char* par)
{
  if (pchannels) pchannels->send_command(command, par);
}
