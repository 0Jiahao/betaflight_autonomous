struct dronerace_vision_struct
{
  int cnt;
  float dx;
  float dy;
  float dz;
};

// store for logging purposes
extern struct dronerace_vision_struct dr_vision;

struct dronerace_state_struct
{
  // Time
  float time;

  // Positon
  float x;
  float y;

  // Speed
  float vx;
  float vy;

  // Heading
  float psi;
};

extern struct dronerace_state_struct dr_state;
extern float ol_dt;

extern void ol_filter_reset(void);

extern void ol_filter_predict(void);
extern void ol_filter_correct(void);
