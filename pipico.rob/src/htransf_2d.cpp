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

#include "htransf_2d.h"

htransf_2d_t::htransf_2d_t()
{
  tx = 0;
  ty = 0;
  theta = 0;

  ct = 1;
  st = 0;
}

htransf_2d_t::htransf_2d_t(float angle, float dx, float dy)
{
  tx = dx;
  ty = dy;
  theta = angle;

  ct = cos(theta);
  st = sin(theta);
}


void htransf_2d_t::set(float angle, float dx, float dy)
{
  tx = dx;
  ty = dy;
  theta = angle;

  ct = cos(theta);
  st = sin(theta);
}

Vec2f htransf_2d_t::apply(Vec2f P)
{
  Vec2f R;
  R.x = ct * P.x - st * P.y + tx;
  R.y = st * P.x + st * P.y + ty;
  return R;
}

htransf_2d_t htransf_2d_t::inverse(void)
{
  htransf_2d_t I;
  I.theta = -theta;
  I.ct =  ct;  //I.ct = cos(I.theta);
  I.st = -st;  //I.st = sin(I.theta);
  I.tx = -ct * tx - st * ty;
  I.ty =  st * tx - ct * ty;
  return I;
}

Vec2f htransf_2d_t::apply_inv(Vec2f P)
{
  Vec2f R;
  R.x = ct * (P.x - tx) + st * (P.y - ty);
  R.y =-st * (P.x - tx) + ct * (P.y - ty);
  return R;
}
