/*
The MIT License (MIT)

Copyright (c) 2024 IMRC Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

// For STAI docs see: https://stedgeai-dc.st.com/assets/embedded-docs/

#include <math.h>
#include <string.h>
#include <inttypes.h>
#include <stdio.h>

#include "math3d.h"
#include "controller_lee.h"
#include "physicalConstants.h"
#include "power_distribution.h"
#include "platform_defaults.h"
#include "param.h"
#include "log.h"
#include "debug.h"
#include "usec_time.h"

#include "network.h"  // Generated model definitions (activations, weights, I/O sizes)
#include "network_data.h"

/* Global handle to reference the instantiated C-model */
static ai_handle network = AI_HANDLE_NULL;

/* Global c-array to handle the activations buffer */
AI_ALIGNED(32)
static ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

/* Array to store the data of the input tensor */
AI_ALIGNED(32)
static ai_float in_data[AI_NETWORK_IN_1_SIZE];
/* or static ai_u8 in_data[AI_NETWORK_IN_1_SIZE_BYTES]; */

/* c-array to store the data of the output tensor */
AI_ALIGNED(32)
static ai_float out_data[AI_NETWORK_OUT_1_SIZE];
/* static ai_u8 out_data[AI_NETWORK_OUT_1_SIZE_BYTES]; */

/* Array of pointer to manage the model's input/output tensors */
static ai_buffer *ai_input;
static ai_buffer *ai_output;


static float lastAction[4] = {0};
static uint32_t rl_print_counter = 0;

#define THRUST_MIN 0.0f     // Minimum thrust (N)
#define THRUST_MAX 0.118f   // Maximum thrust (N)


/* 
 * Bootstrap
 */
int aiInit(void) {
  ai_error err;
  
  /* Create and initialize the c-model */
  const ai_handle acts[] = { activations };
  err = ai_network_create_and_init(&network, acts, NULL);
  if (err.type != AI_ERROR_NONE) { 
    DEBUG_PRINT("AI network creation failed with code: %d\n", err.type);
    return -1;
   };

  /* Reteive pointers to the model's input/output tensors */
  ai_input = ai_network_inputs_get(network, NULL);
  ai_output = ai_network_outputs_get(network, NULL);

  return 0;
}

void controllerRLFirmwareInit(void)
{
  int s = aiInit();
  if (s != 0) {
    DEBUG_PRINT("AI initialization failed with code: %d\n", s);
    return;
  }
}


/* 
 * Run inference
 */
int aiRun(const void *in_data, void *out_data) {
  ai_i32 n_batch;
  ai_error err;
  
  /* 1 - Update IO handlers with the data payload */
  ai_input[0].data = AI_HANDLE_PTR(in_data);
  ai_output[0].data = AI_HANDLE_PTR(out_data);

  /* 2 - Perform the inference */
  n_batch = ai_network_run(network, &ai_input[0], &ai_output[0]);
  if (n_batch != 1) {
      err = ai_network_get_error(network);
      DEBUG_PRINT("AI network run failed with code: %d\n", err.type);
      return -1;
  };
  
  return 0;
}

void controllerRLFirmware(control_t *control,
                          const setpoint_t *setpoint,
                          const sensorData_t *sensors,
                          const state_t     *state,
                          const uint32_t     tick)
{
  if (!RATE_DO_EXECUTE(250, tick)) return;

  /*— compute body-frame errors, velocities, rotation matrix —*/
  struct quat q        = mkquat(state->attitudeQuaternion.x,
                                state->attitudeQuaternion.y,
                                state->attitudeQuaternion.z,
                                state->attitudeQuaternion.w);
  struct mat33 R       = quat2rotmat(q);
  struct vec vel_world = mkvec(state->velocity.x,
                               state->velocity.y,
                               state->velocity.z);
  struct vec vel_body  = qvrot(qinv(q), vel_world);
  struct vec pos_des   = mkvec(setpoint->position.x,
                                setpoint->position.y,
                                setpoint->position.z);
  struct vec pos       = mkvec(state->position.x,
                                state->position.y,
                                state->position.z);
  struct vec pos_err_b = qvrot(qinv(q), vsub(pos_des, pos));

  /*— fill input array —*/
  in_data[0]  = pos_err_b.x;
  in_data[1]  = pos_err_b.y;
  in_data[2]  = pos_err_b.z;
  in_data[3]  = R.m[0][0];
  in_data[4]  = R.m[0][1];
  in_data[5]  = R.m[0][2];
  in_data[6]  = R.m[1][0];
  in_data[7]  = R.m[1][1];
  in_data[8]  = R.m[1][2];
  in_data[9]  = R.m[2][0];
  in_data[10] = R.m[2][1];
  in_data[11] = R.m[2][2];
  in_data[12] = vel_body.x;
  in_data[13] = vel_body.y;
  in_data[14] = vel_body.z;
  in_data[15] = radians(sensors->gyro.x);
  in_data[16] = radians(sensors->gyro.y);
  in_data[17] = radians(sensors->gyro.z);
  in_data[18] = sensors->acc.x;
  in_data[19] = sensors->acc.y;
  in_data[20] = sensors->acc.z;
  in_data[21] = lastAction[0];
  in_data[22] = lastAction[1];
  in_data[23] = lastAction[2];
  in_data[24] = lastAction[3];

    /* 2 - Call inference engine */
    aiRun(in_data, out_data);

    for (int i = 0; i < 4; i++) {
        float t = out_data[i];
        if (t >  1.0f) t =  1.0f;
        if (t < -1.0f) t = -1.0f;
        control->normalizedForces[i] = 0.5f * (t + 1.0f);
        lastAction[i] = t;
    }

    if (++rl_print_counter % 100 == 0) {
        DEBUG_PRINT("action = [%f, %f, %f, %f]\n",
          out_data[0], out_data[1],
          out_data[2], out_data[3]);
        rl_print_counter = 0;
    }

    control->controlMode = controlModeForce;
}

