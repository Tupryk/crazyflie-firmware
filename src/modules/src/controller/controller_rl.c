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

#include "stai.h"     // ST Edge AI Embedded Client API
#include "network.h"  // Generated model definitions (activations, weights, I/O sizes)

static stai_network_t *network = NULL;
static float in_data[STAI_NETWORK_IN_1_SIZE];
static float out_data[STAI_NETWORK_OUT_1_SIZE];
static stai_buffer *stai_input  = NULL;
static stai_buffer *stai_output = NULL;

static float lastAction[4] = {0};
static uint32_t rl_print_counter = 0;

#define THRUST_MIN 0.0f     // Minimum thrust (N)
#define THRUST_MAX 0.118f   // Maximum thrust (N)

void controllerRLFirmwareInit(void)
{
  stai_return_code rc = stai_network_create_and_init(
        &network,
        STAI_NETWORK_DATA_ACTIVATIONS_TABLE_GET(),
        STAI_NETWORK_DATA_WEIGHTS_TABLE_GET()
    );
    if (rc != STAI_SUCCESS) {
        DEBUG_PRINT("stai_network_create_and_init failed: %d\n", rc);
        return;
    }
    stai_input  = stai_network_inputs_get(network,  NULL);
    stai_output = stai_network_outputs_get(network, NULL);
    DEBUG_PRINT("Network initialized via Embedded Client STAI API.\n");
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

    stai_input[0].data  = (stai_ptr)in_data;
    stai_output[0].data = (stai_ptr)out_data;

    stai_error rc = stai_network_run(network, STAI_MODE_SYNC);
    if (rc != STAI_SUCCESS) {
        DEBUG_PRINT("stai_network_run failed: %d\n", rc);
        return;
    }

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

