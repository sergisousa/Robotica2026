/* Copyright (c) 2019  Paulo Costa
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

#include "IRLine.h"
#include "Arduino.h"

IRLine_t::IRLine_t()
{
  sensor_count = 5;
  invert_signal = 1;
  sensor_dist = 16.0; // mm
  IR_WaterLevel = 50;
  line_width = 0;
  IR_tresh = 400;
  cross_tresh = 3;
  black_cross_level = 2.8;
  last_black_time = millis();
}

void IRLine_t::calibrate(void)
{
  
}



void IRLine_t::calcLastBlackTime(void)
{
  int i;
  seing_line = false;
  
  // Update the last time it has seen a black line
  for(i = 0; i < sensor_count; i++) {
    if (IR_values[i] > IR_tresh) {
      last_black_timer.start();
      seing_line = true;
    }
  }
}



void IRLine_t::calcIRLineEdgeLeft(void)
{
  byte c;
  int v, last_v;

  found_left = false;
  IR_max = 0;
  pos_left = 2 * sensor_dist;
  IR_total = 0;
  last_v = 0;
  
  for (c = 0; c < sensor_count; c++) {
    v = IR_values[c] - IR_WaterLevel;
    if (v < 0) v = 0;
    if (v > IR_max) IR_max = v;
    IR_total = IR_total + v;

    if (!found_left && last_v < IR_tresh && v > IR_tresh) {
      pos_left = -line_width/2 + sensor_dist * (c - 3) + sensor_dist * (float)(IR_tresh - last_v) / (float)(v - last_v);
      found_left = true;
    }
    last_v = v;
  }
}


void IRLine_t::calcIRLineEdgeRight(void)
{
  byte c;
  int v, last_v;

  found_right = false;
  IR_max = 0;
  pos_right = -2 * sensor_dist;
  IR_total = 0;
  last_v = 0;
  for (c = 0; c < sensor_count; c++) {
    v = IR_values[sensor_count - 1 - c] - IR_WaterLevel;
    if (v < 0) v = 0;
    if (v > IR_max) IR_max = v;
    IR_total = IR_total + v;

    if (!found_right && last_v < IR_tresh && v > IR_tresh) {
      pos_right = -(-line_width/2 + sensor_dist * (c - 3) + sensor_dist * (IR_tresh - last_v) / (v - last_v));
      found_right = true;
    }
    last_v = v;
  }
  
}



void IRLine_t::calcCrosses(void)
{
  blacks = 0;
  
  if (IR_max <= IR_tresh) {
    cross_count = 0;
    last_cross_count = 0;
    return;
  }
  
  last_cross_count = cross_count;
  
  blacks = IR_total / IR_max;
  if (blacks > black_cross_level) {
    cross_count++;  
    if (cross_count > 32) cross_count = 32;
    if (last_cross_count < cross_tresh && cross_count >= cross_tresh) {
      crosses++;  
    }
  } else {
    if (cross_count > 0) cross_count--;
  }

}


void IRLine_t::calcIRLineCenter(void)
{
  byte c;
  int v;

  found_center = false;
  float sum_pos = 0;
  IR_total = 0;
  for (c = 0; c < sensor_count; c++) {
    v = IR_values[c] - IR_WaterLevel;
    if (v < 0) v = 0;
 
    IR_total = IR_total + v;
    sum_pos = sum_pos + v * (c - trunc(sensor_count / 2)) * sensor_dist;
  }

  if (IR_total > IR_tresh) {  // At least one sensor has seen black
    pos_center = sum_pos / IR_total;
    found_center = true;
  } // pos center is not updated if the line is not seen
}

void IRLine_t::calcIRLineCenterDigital(void)
{
  byte c;
  int v;

  found_center_digital = false;
  float sum_pos = 0;
  IR_total = 0;
  for (c = 0; c < sensor_count; c++) {
    v = 0;
    if (IR_values[c] > IR_tresh) v = 1;
 
    IR_total = IR_total + v;
    sum_pos = sum_pos + v * (c + 1) * sensor_dist;
  }

  if (IR_total > 0) {  // At least one sensor has seen black
    pos_center_digital = sum_pos / IR_total - sensor_dist * (0.5 * (sensor_count + 1));
    found_center_digital = true;
  } // pos center is not updated if the line is not seen
}

