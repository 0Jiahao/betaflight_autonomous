#include <math.h>

#include "flight/ol_control.h"
#include "flight/ol_filter.h"
#include "flight/ol_flightplan.h"
#include "flight/imu.h"
#include "build/debug.h"

// Variables
struct dronerace_control_struct dr_control;

// Settings


// Slow speed
#define DR_FILTER_GRAVITY  9.81
#define CTRL_MAX_SPEED  2.5             // m/s
#define CTRL_MAX_PITCH  15.0 / 180.0 * 3.1415926f   // deg
#define CTRL_MAX_ROLL   10.0 / 180.0 * 3.1415926f  // deg
#define CTRL_MAX_R      45.0 / 180.0 * 3.1415926f // deg/sec

/*
// Max speed for bebop
#define CTRL_MAX_SPEED  6.0             // m/s
#define CTRL_MAX_PITCH  RadOfDeg(35)    // rad
#define CTRL_MAX_ROLL   RadOfDeg(35)    // rad
#define CTRL_MAX_R      RadOfDeg(45)    // rad/sec
*/

/*
// Race drone
#define CTRL_MAX_SPEED  10              // m/s
#define CTRL_MAX_PITCH  RadOfDeg(45)    // rad
#define CTRL_MAX_ROLL   RadOfDeg(45)    // rad
#define CTRL_MAX_R      RadOfDeg(180)   // rad/sec
*/

static inline float ol_DEGREES_TO_RADIANS(float angle)
{
    return (angle) * 0.0174532925f;
}

static inline float ol_constrainf(float amt, float low, float high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}

void ol_control_reset(void)
{
  // Reset flight plan logic
  ol_flightplan_reset();

  // Reset own variables
  dr_control.psi_ref = 0;
}

void ol_control_run(void)
{
  float dt = ol_dt;
  float psi, vxcmd, vycmd, r_cmd, ax, ay;
  // Propagate the flightplan
  ol_flightplan_run();

  // Variables
  psi = dr_state.psi;

  // Heading controller
  r_cmd = dr_fp.psi_set - dr_control.psi_ref;

  // Find shortest turn
  if(r_cmd < -3.1415926f)
  {
      r_cmd += 2 * 3.1415926f;
  }
  else if(r_cmd > 3.1415926f)
  {
      r_cmd -= 2 * 3.1415926f;
  }

  // Apply rate limit
  r_cmd = ol_constrainf(r_cmd, -CTRL_MAX_R, CTRL_MAX_R);
  dr_control.psi_ref += r_cmd * dt;
  DEBUG_SET(DEBUG_PSI,0,dr_fp.gate_nr);
  DEBUG_SET(DEBUG_PSI,1,dr_control.psi_ref/3.14*180);
  DEBUG_SET(DEBUG_PSI,2,dr_fp.psi_set/3.14*180);
  DEBUG_SET(DEBUG_PSI,3,attitude.values.yaw/10);
  // Position error to Speed
  vxcmd = (dr_fp.x_set - dr_state.x) * 1.1f - dr_state.vx * 0.0f;
  vycmd = (dr_fp.y_set - dr_state.y) * 1.1f - dr_state.vy * 0.0f;

  vxcmd = ol_constrainf(vxcmd, -CTRL_MAX_SPEED, CTRL_MAX_SPEED);
  vycmd = ol_constrainf(vycmd, -CTRL_MAX_SPEED, CTRL_MAX_SPEED);

  // Speed to Attitude
  ax = (vxcmd - dr_state.vx) * 1.0f + vxcmd * ol_DEGREES_TO_RADIANS(10.0f) / 3.0f;
  ay = (vycmd - dr_state.vy) * 1.0f + vycmd * ol_DEGREES_TO_RADIANS(10.0f) / 3.0f;

  ax = ol_constrainf(ax, -CTRL_MAX_PITCH, CTRL_MAX_PITCH);
  ay = ol_constrainf(ay, -CTRL_MAX_PITCH, CTRL_MAX_PITCH);

  DEBUG_SET(DEBUG_CA,0,ax/3.14*180);
  DEBUG_SET(DEBUG_CA,1,ay/3.14*180);
  DEBUG_SET(DEBUG_CA,2, dr_control.theta_cmd/3.14*180);
  DEBUG_SET(DEBUG_CA,3, dr_control.phi_cmd/3.14*180);

  dr_control.phi_cmd   = - sinf(psi) * ax + cosf(psi) * ay; //rad
  dr_control.theta_cmd = - cosf(psi) * ax - sinf(psi) * ay; //rad

  dr_control.psi_cmd   = dr_control.psi_ref;                //rad
  dr_control.alt_cmd   = dr_fp.alt_set;                     //m
}
