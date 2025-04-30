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
#include "network_data_params.h"

static ai_handle network;
static ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];
static float aiInData[AI_NETWORK_IN_1_SIZE];
static float aiOutData[AI_NETWORK_OUT_1_SIZE];
static float lastAction[4] = {0};

static ai_buffer ai_input[AI_NETWORK_IN_1_NB_BUFFERS];
static ai_buffer ai_output[AI_NETWORK_OUT_1_NB_BUFFERS];

#define THRUST_MIN 0.0f     // Minimum thrust (N)
#define THRUST_MAX 0.118f   // Maximum thrust (N)

void controllerRLFirmwareInit(void)
{
  /* Create a local array with the addresses of the activations buffers */
  const ai_handle act_addr[] = { activations };
  ai_error e = ai_network_create_and_init(&network, act_addr, NULL);
  if (e.type != AI_ERROR_NONE)
  {
    DEBUG_PRINT("Failed to initialize network. Error code: %d.%d\n", e.type, e.code);
  }
  else
  {
    DEBUG_PRINT("Neural network initialized successfully.\n");
  }
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

// pos_error, # (3,) 0:3
// quad1_rot, # (9,) 3:12
// quad1_linvel, # (3,) 12:15
// quad1_angvel, # (3,) 15:18
// quad1_linear_acc, # (3,) 18:21
// quad1_angular_acc, # (3,) 21:24
// last_action, # (4,) 24:28

  aiInData[0]  = pos_error_body.x;
  aiInData[1]  = pos_error_body.y;
  aiInData[2]  = pos_error_body.z;
  aiInData[3]  = R.m[0][0];
  aiInData[4]  = R.m[0][1];
  aiInData[5]  = R.m[0][2];
  aiInData[6]  = R.m[1][0];
  aiInData[7]  = R.m[1][1];
  aiInData[8]  = R.m[1][2];
  aiInData[9]  = R.m[2][0];
  aiInData[10] = R.m[2][1];
  aiInData[11] = R.m[2][2];
  aiInData[12] = vel_body.x;
  aiInData[13] = vel_body.y;
  aiInData[14] = vel_body.z;
  aiInData[15] = radians(sensors->gyro.x);
  aiInData[16] = radians(sensors->gyro.y);
  aiInData[17] = radians(sensors->gyro.z);
  aiInData[18] = sensors->acc.x;
  aiInData[19] = sensors->acc.y;
  aiInData[20] = sensors->acc.z;
  aiInData[21] = lastAction[0];
  aiInData[22] = lastAction[1];
  aiInData[23] = lastAction[2];
  aiInData[24] = lastAction[3];

  ai_network_inputs_get(network,  &ai_input[0]);
  ai_network_outputs_get(network, &ai_output[0]);

  ai_input[0].data        = AI_HANDLE_PTR(aiInData);
  ai_input[0].data_start  = AI_HANDLE_PTR(aiInData);
  ai_input[0].n_batches   = 1;

  ai_output[0].data        = AI_HANDLE_PTR(aiOutData);
  ai_output[0].data_start  = AI_HANDLE_PTR(aiOutData);
  ai_output[0].n_batches   = 1;

  // Run neural network
  uint64_t start = usecTimestamp();
  ai_i32 batch = ai_network_run(network, &ai_input[0], &ai_output[0]);
  uint64_t end = usecTimestamp();

  if (batch != 1)
  {
    ai_error err = ai_network_get_error(network);
    DEBUG_PRINT("Inference failed. Error code: %d.%d\n", err.type, err.code);
    return;
  }

  for (int i = 0; i < 4; i++)
  {
    // clip to [-1,1]
    if (aiOutData[i] > 1.0f)      aiOutData[i] = 1.0f;
    else if (aiOutData[i] < -1.0f) aiOutData[i] = -1.0f;
    // Thrust commands for four motors mapped to [0, 0.118] (N)
    // control->thrust[4-i] = THRUST_MIN + (0.5f * (aiOutData[i]
    control->normalizedForces[i] = 0.5f * (aiOutData[i] + 1.0f);
    lastAction[i] = aiOutData[i];
    control->controlMode = controlModeForce;
  }
  
}

