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

#include "math3d.h"
#include "controller_lee.h"
#include "physicalConstants.h"
#include "power_distribution.h"
#include "platform_defaults.h"
#include "param.h"
#include "log.h"
#include "debug.h"
#include "usec_time.h"

#include "stai.h"             /* ST Edge AI runtime & APIs */
#include "network.h"          /* Generated model macros */

/* Declare and allocate private network context buffer */
STAI_NETWORK_CONTEXT_DECLARE(stai_network_ctx, STAI_NETWORK_CONTEXT_SIZE);

/* Buffers for I/O */
STAI_ALIGNED(STAI_NETWORK_IN_1_ALIGNMENT)
static float in_data[STAI_NETWORK_IN_1_SIZE];
STAI_ALIGNED(STAI_NETWORK_OUT_1_ALIGNMENT)
static float out_data[STAI_NETWORK_OUT_1_SIZE];

static float lastAction[4] = {0};
static uint32_t rl_print_counter = 0;

#define THRUST_MIN 0.0f     // Minimum thrust (N)
#define THRUST_MAX 0.118f   // Maximum thrust (N)

void controllerRLFirmwareInit(void)
{
  stai_return_code rc;

  /* Initialize the ST Edge AI C runtime */
  rc = stai_runtime_init();
  if (rc != STAI_SUCCESS) {
    DEBUG_PRINT("Failed to init STAI runtime: 0x%x\n", rc);
    return;
  }

  /* Initialize network context */
  rc = stai_network_init(stai_network_ctx);
  if (rc != STAI_SUCCESS) {
    DEBUG_PRINT("Failed to init network: 0x%x\n", rc);
    return;
  }

  /* Allocate and bind activations */
  stai_ptr activation_buffers[STAI_NETWORK_ACTIVATIONS_NUM] = {0};
  STAI_ALIGNED(STAI_NETWORK_ACTIVATION_1_ALIGNMENT)
  uint8_t activation1[STAI_NETWORK_ACTIVATION_1_SIZE] = {0};
  STAI_ALIGNED(STAI_NETWORK_ACTIVATION_2_ALIGNMENT)
  uint8_t activation2[STAI_NETWORK_ACTIVATION_2_SIZE] = {0};
  activation_buffers[0] = (stai_ptr)activation1;
  activation_buffers[1] = (stai_ptr)activation2;

  rc = stai_network_set_activations(stai_network_ctx,
                                    activation_buffers,
                                    STAI_NETWORK_ACTIVATIONS_NUM);
  if (rc != STAI_SUCCESS) {
    DEBUG_PRINT("Failed to set activations: 0x%x\n", rc);
    return;
  }
}

void controllerRLFirmware(control_t *control,
                          const setpoint_t *setpoint,
                          const sensorData_t *sensors,
                          const state_t     *state,
                          const uint32_t     tick)
{
  stai_return_code rc;

  /* throttle to 250 Hz */
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

  /* Bind input */
  stai_ptr input_buffers[STAI_NETWORK_IN_NUM] = {(stai_ptr)in_data};
  rc = stai_network_set_inputs(stai_network_ctx,
                               input_buffers,
                               STAI_NETWORK_IN_NUM);
  if (rc != STAI_SUCCESS) {
    DEBUG_PRINT("Failed to set inputs: 0x%x\n", rc);
    return;
  }

  /* Bind output */
  stai_ptr output_buffers[STAI_NETWORK_OUT_NUM] = {(stai_ptr)out_data};
  rc = stai_network_set_outputs(stai_network_ctx,
                                output_buffers,
                                STAI_NETWORK_OUT_NUM);
  if (rc != STAI_SUCCESS) {
    DEBUG_PRINT("Failed to set outputs: 0x%x\n", rc);
    return;
  }

  /* Run inference */
  rc = stai_network_run(stai_network_ctx, STAI_MODE_SYNC);
  if (rc != STAI_SUCCESS) {
    DEBUG_PRINT("Inference failed: 0x%x\n", rc);
    return;
  }

  /* Post-process outputs */
  for (int i = 0; i < 4; i++) {
    float t = out_data[i];
    if (t >  1.0f) t =  1.0f;
    if (t < -1.0f) t = -1.0f;
    control->normalizedForces[i] = 0.5f * (t + 1.0f);
    lastAction[i] = t;
  }

  if (++rl_print_counter % 100 == 0) {
    DEBUG_PRINT("action = [%f, %f, %f, %f]\n",
      lastAction[0], lastAction[1],
      lastAction[2], lastAction[3]);
    rl_print_counter = 0;
  }

  control->controlMode = controlModeForce;
}

