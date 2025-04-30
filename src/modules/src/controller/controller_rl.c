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

// For STAI docs see: https://stedgeai-dc.st.com/documentation

#include <math.h>
#include <string.h>

#include "math3d.h"
#include "controller_lee.h"
#include "physicalConstants.h"
#include "power_distribution.h"
#include "platform_defaults.h"
#include "param.h"
#include "log.h"
#include "debug.h"
#include "usec_time.h"

#include "network.h"
#include "network_data.h"
#include "network_data_params.h"

static ai_handle rl_network = AI_HANDLE_NULL;


static AI_ALIGNED(4)
ai_u8 rl_activations_data[ AI_NETWORK_DATA_ACTIVATIONS_SIZE_BYTES ];

static const ai_handle rl_activations[] = {
  AI_NETWORK_DATA_ACTIVATIONS(rl_activations_data)
};


extern const ai_u64 s_network_weights_array_u64[ AI_NETWORK_DATA_WEIGHTS_COUNT * (AI_NETWORK_DATA_WEIGHTS_SIZE/8) ];
static const ai_handle rl_weights[] = {
  AI_NETWORK_DATA_WEIGHTS(s_network_weights_array_u64)
};

static ai_buffer *input_bufs  = NULL;
static ai_buffer *output_bufs = NULL;

static float lastAction[4] = {0};
static uint32_t rl_print_counter = 0;

#define THRUST_MIN 0.0f     // Minimum thrust (N)
#define THRUST_MAX 0.118f   // Maximum thrust (N)

void controllerRLFirmwareInit(void)
{
  ai_error err = ai_network_create_and_init(
    &rl_network,
    rl_activations,
    rl_weights
  );
  if (err.type != AI_ERROR_NONE) {
    DEBUG_PRINT("AI network init failed: %d\n", err.type);
    return;
  }
  input_bufs  = ai_network_inputs_get(rl_network,  NULL);
  output_bufs = ai_network_outputs_get(rl_network, NULL);
  DEBUG_PRINT("AI network initialized successfully.\n");
}

bool controllerRLFirmwareTest(void)
{
  return true;
}

void controllerRLFirmware(control_t *control, const setpoint_t *setpoint,
                          const sensorData_t *sensors, const state_t *state,
                          const uint32_t tick)
{
  // TODO: limit to only 1 Hz for debugging
  if (!RATE_DO_EXECUTE(250, tick)) {
    return;
  }

  struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
  struct mat33 R = quat2rotmat(q);

  // velocity in body frame
  struct vec vel_world = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);
  struct vec vel_body = qvrot(qinv(q), vel_world);

  // angular_velocity in body frame
  struct vec pos_desired = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
  struct vec pos = mkvec(state->position.x, state->position.y, state->position.z);
  struct vec pos_error_world = vsub(pos_desired, pos);
  struct vec pos_error_body  = qvrot(qinv(q), pos_error_world);

  float in_buf[25];
  in_buf[0]  = pos_error_body.x;
  in_buf[1]  = pos_error_body.y;
  in_buf[2]  = pos_error_body.z;
  in_buf[3]  = R.m[0][0];
  in_buf[4]  = R.m[0][1];
  in_buf[5]  = R.m[0][2];
  in_buf[6]  = R.m[1][0];
  in_buf[7]  = R.m[1][1];
  in_buf[8]  = R.m[1][2];
  in_buf[9]  = R.m[2][0];
  in_buf[10] = R.m[2][1];
  in_buf[11] = R.m[2][2];
  in_buf[12] = vel_body.x;
  in_buf[13] = vel_body.y;
  in_buf[14] = vel_body.z;
  in_buf[15] = radians(sensors->gyro.x);
  in_buf[16] = radians(sensors->gyro.y);
  in_buf[17] = radians(sensors->gyro.z);
  in_buf[18] = sensors->acc.x;
  in_buf[19] = sensors->acc.y;
  in_buf[20] = sensors->acc.z;
  in_buf[21] = lastAction[0];
  in_buf[22] = lastAction[1];
  in_buf[23] = lastAction[2];
  in_buf[24] = lastAction[3];

  // copy into AI input buffer
  memcpy(input_bufs[0].data, in_buf, sizeof(in_buf));

  // run AI inference
  ai_i32 nb = ai_network_run(rl_network, input_bufs, output_bufs);
  if (nb <= 0) {
    DEBUG_PRINT("AI inference failed\n");
    return;
  }

  rl_print_counter++;

  for (int i = 0; i < 4; i++) {
    float t = ((float*)output_bufs[0].data)[i];
    if (t >  1.0f) t =  1.0f;
    if (t < -1.0f) t = -1.0f;
    control->normalizedForces[i] = 0.5f * (t + 1.0f);
    lastAction[i] = t;

    if (rl_print_counter % 100 == 0) {
      DEBUG_PRINT("action[%d] = %f\n", i, t);
    }
  }

  if (rl_print_counter >= 100) {
    rl_print_counter = 0;
  }

  control->controlMode = controlModeForce;
}

