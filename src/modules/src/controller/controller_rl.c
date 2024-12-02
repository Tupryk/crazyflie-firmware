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

static ai_buffer * ai_input;
static ai_buffer * ai_output;

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
  ai_input = ai_network_inputs_get(network, NULL);
  ai_output = ai_network_outputs_get(network, NULL);
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
  if (!RATE_DO_EXECUTE(200, tick)) {
    return;
  }

  struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
  struct mat33 R = quat2rotmat(q);

  // flat rotation matrix
  aiInData[0] = R.m[0][0];
  aiInData[1] = R.m[0][1];
  aiInData[2] = R.m[0][2];
  aiInData[3] = R.m[1][0];
  aiInData[4] = R.m[1][1];
  aiInData[5] = R.m[1][2];
  aiInData[6] = R.m[2][0];
  aiInData[7] = R.m[2][1];
  aiInData[8] = R.m[2][2];

  // velocity in body frame
  struct vec vel_world = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);
  struct vec vel_body = qvrot(qinv(q), vel_world);
  aiInData[9]  = vel_body.x;
  aiInData[10] = vel_body.y;
  aiInData[11] = vel_body.z;

  // angular_velocity in body frame
  aiInData[12] = radians(sensors->gyro.x);
  aiInData[13] = radians(sensors->gyro.y);
  aiInData[14] = radians(sensors->gyro.z);

  // position error in body frame
  struct vec pos_desired = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
  struct vec pos = mkvec(state->position.x, state->position.y, state->position.z);
  struct vec pos_error_world = vsub(pos_desired, pos);

  struct vec pos_error_body = qvrot(qinv(q), pos_error_world);

  aiInData[15] = pos_error_body.x;
  aiInData[16] = pos_error_body.y;
  aiInData[17] = pos_error_body.z;

  // Bind input and output buffers
  ai_input[0].data = AI_HANDLE_PTR(aiInData);
  ai_output[0].data = AI_HANDLE_PTR(aiOutData);

  // Run neural network
  uint64_t start = usecTimestamp();
  ai_i32 batch = ai_network_run(network, ai_input, ai_output);
  uint64_t end = usecTimestamp();

  if (batch != 1)
  {
    ai_error err = ai_network_get_error(network);
    DEBUG_PRINT("Inference failed. Error code: %d.%d\n", err.type, err.code);
    return;
  }

  // DEBUG_PRINT("%llu, %f, %f, %f, %f\n", end-start, (float)aiOutData[0], (float)aiOutData[1], (float)aiOutData[2], (float)aiOutData[3]);

  // Map nn output to control commands
  for (int i = 0; i < 4; i++)
  {
    // clip thrust commands to [-1 1]
    if (aiOutData[i] > 1.0f)
    {
      aiOutData[i] = 1.0f;
    }
    else if (aiOutData[i] < -1.0f)
    {
      aiOutData[i] = -1.0f;
    }

    // Thrust commands for four motors mapped to [0, 0.118] (N)
    // control->thrust[4-i] = THRUST_MIN + (0.5f * (aiOutData[i] + 1.0f)) * (THRUST_MAX - THRUST_MIN);
    control->normalizedForces[3-i] =0.5f * (aiOutData[i] + 1.0f);
    control->controlMode = controlModeForce;
  }

  // mojoco output order: m4, m3, m2, m1
  // 

  // // Run inference
  // if (runInference())
  // {
  //   // Map nn output to control commands
  //   for (int i = 0; i < 4; i++)
  //   {
  //     // clip thrust commands to [-1 1]
  //     if (action_output[i] > 1.0f)
  //     {
  //       action_output[i] = 1.0f;
  //     }
  //     else if (action_output[i] < -1.0f)
  //     {
  //       action_output[i] = -1.0f;
  //     }

  //     // Thrust commands for four motors mapped to [0, 0.118] (N)
  //     // control->thrust[i] = THRUST_MIN + (0.5f * (action_output[i] + 1.0f)) * (THRUST_MAX - THRUST_MIN);


      
      
  //     // TODO: apply actions properly. I think we need to scale and clip the action.
  //     // python:
  //     // # Post - process action
  //     // # Rescale the action from[-1, 1] to[low, high]
  //     // low, high = env.action_space.low, env.action_space.high action = low + (0.5 * (scaled_action + 1.0) * (high - low)) 
  //     // action = np.clip(action, low, high)

  //     // Thrust commands for four motors clipped to [0, 0.118] (N)
  //     // 
  //     // mapping: 
  //     // 4 - 1
  //     // |   |
  //     // 3 - 2
  //     //<site name="thrust1" pos="0.032527 0.032527 0"/>  => M4
  //     // <site name="thrust2" pos="-0.032527 0.032527 0"/> => M3
  //     // <site name="thrust3" pos="-0.032527 -0.032527 0"/> => M2
  //     // <site name="thrust4" pos="0.032527 -0.032527 0"/> => M1
  //   }
  // }
  // else
  // {
  //   DEBUG_PRINT("Inference Error. Not updating controller commands.");
  // }

}

