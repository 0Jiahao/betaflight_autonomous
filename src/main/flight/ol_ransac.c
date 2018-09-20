#include "flight/ol_ransac.h"
#include "flight/ol_filter.h"

#define  RANSAC_BUF_SIZE   20

// Time, x_predict, y, x_measured, y
struct dronerace_ransac_buf_struct
{
  // Settings
  float time;

  // Predicted States
  float x;
  float y;

  // Measured States
  float mx;
  float my;
};

struct dronerace_ransac_buf_struct ransac_buf[RANSAC_BUF_SIZE];

struct dronerace_ransac_struct dr_ransac;


void ransac_reset(void)
{
  int i;
  for (i=0; i<RANSAC_BUF_SIZE; i++) {
    ransac_buf[i].time = 0;
    ransac_buf[i].x = 0;
    ransac_buf[i].y = 0;
    ransac_buf[i].mx = 0;
    ransac_buf[i].my = 0;
  }
  dr_ransac.dt_max = 1.0;
}

// From newest (0) to oldest (RANSAC_BUF_SIZE)
static int get_index(int element)
{
  int ind = dr_ransac.buf_index_of_last - element;
  if (ind < 0) { ind += RANSAC_BUF_SIZE; }
  return ind;
}

void ransac_update_buffer_size(void)
{
  int i;

  // Update buffer size
  dr_ransac.buf_size = 0;
  for (i=0; i<RANSAC_BUF_SIZE; i++ )
  {
    float mt = ransac_buf[get_index(i)].time;
    if ((mt == 0) || ((dr_state.time - mt) > dr_ransac.dt_max))
    {
      break;
    }
    dr_ransac.buf_size++;
  }
}

void ransac_push(float time, float x, float y, float mx, float my)
{
  int i = 0;
  static int ransac_cnt = 0;

  // Insert in the buffer
  dr_ransac.buf_index_of_last++;
  if (dr_ransac.buf_index_of_last >= RANSAC_BUF_SIZE)
  {
    dr_ransac.buf_index_of_last = 0;
  }
  ransac_buf[dr_ransac.buf_index_of_last].time = time;
  ransac_buf[dr_ransac.buf_index_of_last].x = x;
  ransac_buf[dr_ransac.buf_index_of_last].y = y;
  ransac_buf[dr_ransac.buf_index_of_last].mx = mx;
  ransac_buf[dr_ransac.buf_index_of_last].my = my;

//   ransac_update_buffer_size();


  // If sufficient items in buffer
  if (dr_ransac.buf_size > 4)
  {
    /** Perform RANSAC to fit a linear model.
     *
     * @param[in] n_samples The number of samples to use for a single fit
     * @param[in] n_iterations The number of times a linear fit is performed
     * @param[in] error_threshold The threshold used to cap errors in the RANSAC process
     * @param[in] targets The target values
     * @param[in] samples The samples / feature vectors
     * @param[in] D The dimensionality of the samples
     * @param[in] count The number of samples
     * @param[out] parameters* Parameters of the linear fit
     * @param[out] fit_error* Total error of the fit
     */

    int n_samples = ((int)dr_ransac.buf_size * 0.4);
    int n_iterations = 200;
    float error_threshold = 1.0;
    int D = 1;
    int count = dr_ransac.buf_size;
    float targets_x[count];
    float targets_y[count];
    float samples[count][1];
    float params_x[2];
    float params_y[2];
    float fit_error;

    for (i=0;i<count;i++)
    {
      struct dronerace_ransac_buf_struct* r = &ransac_buf[get_index(i)];
      targets_x[i] = r->x - r->mx;
      targets_y[i] = r->y - r->my;
      samples[i][0] = r->time - dr_state.time;
    //   printf("Fit %f to %f\n", targets_x[i], samples[i][0]);
    }

    // printf("Running RANSAC with %d points and %d samples, err_max %f\n",count, n_samples, error_threshold);


    // RANSAC_linear_model( n_samples, n_iterations,  error_threshold,
    //     targets_x, D, samples, count, params_x, &fit_error);

    //RANSAC_linear_model( n_samples, n_iterations,  error_threshold, targets_y, D, samples, count, params_y, &fit_error);

    ransac_cnt ++;



// #define DEBUG_RANSAC
// #ifdef DEBUG_RANSAC
//     {
//       char filename[128];
//       sprintf(filename,"debug/ransac%06d.txt",ransac_cnt);
//       FILE* fp = fopen(filename,"w");
//       fprintf(fp,"time,x,y,mx,my\n");
//       //printf("t=%f %f\n\n",dr_state.time,dr_ransac.dt_max);
//       for (i=0;i<RANSAC_BUF_SIZE;i++)
//       {
//         fprintf(fp,"%d,%f,%f,%f,%f,%f\n",i,ransac_buf[get_index(i)].time,
//             ransac_buf[get_index(i)].x,
//             ransac_buf[get_index(i)].y,
//             ransac_buf[get_index(i)].mx,
//             ransac_buf[get_index(i)].my
//             );
//       }

//       fprintf(fp,"\n\n X = %f %f Y = %f  %f \n",params_x[0], params_x[1], params_y[0], params_y[1] );

//       fclose(fp);
//     }
//     exit(-1);
// #endif

  }
}

#include <stdio.h>

static void ransac_get_vector(void)
{
  int i = 0;
  for ( i=0; i<dr_ransac.buf_size; i++ )
  {
    // Get index
    ransac_buf[get_index(i)];
  }
}
