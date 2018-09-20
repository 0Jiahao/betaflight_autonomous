#include <math.h>

#include "drivers/time.h"

#include "flight/ol_control.h"
#include "flight/ol_flightplan.h"
#include "flight/ol_filter.h"
#include "flight/imu.h"

#include "build/debug.h"

struct dronerace_state_struct dr_state;
struct dronerace_vision_struct dr_vision;
float ol_dt = 0.0005;

void ol_filter_reset()
{
  // Time
  dr_state.time = 0;

  // Position
  dr_state.x = 0;
  dr_state.y = 0;

  // Speed
  dr_state.vx = 0;
  dr_state.vy = 0;

  // Heading
  dr_state.psi = 0;

  // Vision latency
//   fifo_reset();
//   ransac_reset();
}

// PREDICTION MODEL
#define DR_FILTER_GRAVITY  9.81
#define DR_FILTER_DRAG  0.5
#define DR_FILTER_THRUSTCORR  0.8

void ol_filter_predict(void)
{
  ////////////////////////////////////////////////////////////////////////
  // Read attitude
  float theta = -DECIDEGREES_TO_RADIANS(attitude.values.pitch);
  float phi = DECIDEGREES_TO_RADIANS(attitude.values.roll);
  float psi = DECIDEGREES_TO_RADIANS(attitude.values.yaw);

  timeUs_t currentTimeUs = micros();
  static timeUs_t previousTimeUs;
  const timeDelta_t deltaT = currentTimeUs - previousTimeUs;
  ol_dt = deltaT * 1e-6f;
  previousTimeUs = currentTimeUs;

  // Body accelerations
  float abx =  sinf(-theta) * DR_FILTER_GRAVITY / cosf(theta * DR_FILTER_THRUSTCORR) / cosf(phi * DR_FILTER_THRUSTCORR);
  float aby =  sinf( phi)   * DR_FILTER_GRAVITY / cosf(theta * DR_FILTER_THRUSTCORR) / cosf(phi * DR_FILTER_THRUSTCORR);
  
  // Earth accelerations
  float ax =  cosf(psi) * abx - sinf(psi) * aby - dr_state.vx * DR_FILTER_DRAG ;
  float ay =  sinf(psi) * abx + cosf(psi) * aby - dr_state.vy * DR_FILTER_DRAG;


  // Velocity and Position
  dr_state.vx += ax * ol_dt;
  dr_state.vy += ay * ol_dt;
  dr_state.x += dr_state.vx * ol_dt;
  dr_state.y += dr_state.vy * ol_dt;
  DEBUG_SET(DEBUG_OL,0, 100 * dr_state.x);
  DEBUG_SET(DEBUG_OL,1, 100 * dr_state.y);
  DEBUG_SET(DEBUG_OL,2, 100 * dr_state.vx);
  DEBUG_SET(DEBUG_OL,3, 100 * dr_state.vy);
  DEBUG_SET(DEBUG_FP,0, 100 * dr_state.x);
  DEBUG_SET(DEBUG_FP,1, 100 * dr_state.y);
  DEBUG_SET(DEBUG_FP,2, dr_control.theta_cmd/3.14*180);
  DEBUG_SET(DEBUG_FP,3, dr_control.phi_cmd/3.14*180);

  // Time
  dr_state.time += ol_dt;

  // Store psi for local corrections
  dr_state.psi = psi;

  // Store old states for latency compensation
//   fifo_push(dr_state.x, dr_state.y, 0);

  // Check if Ransac buffer is empty
  ransac_update_buffer_size();
}

void ol_filter_correct(void)
{
  // Retrieve oldest element of state buffer (that corresponds to current vision measurement)
  float sx, sy, sz;
  float mx, my;

//   fifo_pop(&sx, &sy, &sz);

  // Compute current absolute position
  // TODO: check: this is probably wrong!
  mx = dr_fp.gate_x - dr_vision.dx;
  my = dr_fp.gate_y - dr_vision.dy;

  // Push to RANSAC
//   ransac_push(dr_state.time, dr_state.x, dr_state.y, mx, my);
}