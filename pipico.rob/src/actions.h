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

#ifndef actions_H
#define actions_H

#include "Arduino.h"
#include "robot.h"
#include "graph.h"
#include <VectorXf.h>

enum action_state_t {
  as_stop, // 0
  as_set_theta,
  as_goto_xy, // 2
  as_follow_line, // 3
  as_follow_circle, // 4
  as_follow_track,
  as_gollow_track_digital,
  as_follow_track_left,
  as_follow_track_right,
  as_follow_wall_right,
  as_follow_wall_left,
  as_robot_at_factory // 11
};

class action_t
{
  public:
    Vec2f Pi;
    Vec2f Pf;
    float thetai;
    float thetaf;
    float v_nom;
    Vec2f C;
    float alpha, radius;

    float e_xy, e_theta;
    float e_xy_tresh, e_theta_tresh;

    float kset_theta;
    float ktheta, kn;
    float ktrack, wz;
    float w0;
    float wall_dist_left_ref;
    float wall_dist_right_ref;

    int path[N_nodes];
    int idx_path;

    bool done;
    bool stop_at_end;
    bool robotatfactory;
    bool traj_done;

    action_t();

    void stop(void);
    void set_theta(void);
    void goto_xy(void);
    void follow_line(void);
    void follow_circle(void);
    void follow_track(void);
    void follow_track_digital(void);

    void follow_track_right(void);
    void follow_track_left(void);
    void follow_wall_right(void);
    void follow_wall_left(void);

    void do_action_list(void);
    void do_action(void);

    void update_mean_abs_w(void);
};


float sqr(float x);
float sign(float x);
float norm(float x, float y);
void normalize(float& x, float& y);

float dist(float x0, float y0, float x1, float y1);
float normalize_angle(float ang);
float dif_angle(float a0, float a1);

extern action_t action;

class segment_t{
  public:
    Vec2f Pi;
    Vec2f Pf;
    double angle, dist;
    segment_t();
    segment_t(Vec2f thePi, Vec2f thePf);

    void refresh(void);
    void set_points(float xi, float yi, float xf, float yf);
};


#endif // actions_H
