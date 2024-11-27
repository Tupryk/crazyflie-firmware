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

#include "network.h"
#include "network_data_params.h"

static ai_handle network;

// Neural network input and output buffers
static ai_buffer ai_input[AI_NETWORK_IN_NUM];
static ai_buffer ai_output[AI_NETWORK_OUT_NUM];

// Observation variables
static float orientation_matrix[9]; // Orientation matrix (3x3)
static float linear_velocity[3];    // Linear velocity in drone's local frame (x, y, z)
static float angular_velocity[3];   // Angular velocity in drone's local frame (roll, pitch, yaw rates)
static float position_error[3];     // Position error in drone's local frame (x, y, z)

// Action outputs
static float action_output[4]; // Thrust commands for four motors

void controllerRLFirmwareInit(void)
{
  ai_error e = ai_network_create_and_init(&network, AI_NETWORK_DATA_ACTIVATIONS_TABLE_GET(), AI_NETWORK_DATA_WEIGHTS_TABLE_GET());
  if (e.type != AI_ERROR_NONE)
  {
    LOG_ERROR("Failed to initialize network. Error code: %d", e.code);
  }
  else
  {
    LOG_INFO("Neural network initialized successfully.");
  }

  // Initialize input and output buffers
  ai_input[0].data = AI_HANDLE_PTR(NULL);  // Assign null initially
  ai_output[0].data = AI_HANDLE_PTR(NULL); // Assign null initially
}

bool runInference()
{
  // Fill input data
  float input_data[18];
  int idx = 0;

  // Flat orientation matrix
  for (int i = 0; i < 9; i++)
  {
    input_data[idx++] = orientation_matrix[i];
  }

  // Add linear velocity
  for (int i = 0; i < 3; i++)
  {
    input_data[idx++] = linear_velocity[i];
  }

  // Add angular velocity
  for (int i = 0; i < 3; i++)
  {
    input_data[idx++] = angular_velocity[i];
  }

  // Add position error
  for (int i = 0; i < 3; i++)
  {
    input_data[idx++] = position_error[i];
  }

  // Bind input and output buffers
  ai_input[0].data = AI_HANDLE_PTR(input_data);
  ai_output[0].data = AI_HANDLE_PTR(action_output);

  // Run neural network
  ai_i32 batch = ai_network_run(network, ai_input, ai_output);
  if (batch != 1)
  {
    ai_error err = ai_network_get_error(network);
    LOG_ERROR("Inference failed. Error code: %d", err.code);
    return false;
  }
  return true;
}

void controllerRLFirmware(control_t *control, const setpoint_t *setpoint,
                          const sensorData_t *sensors, const state_t *state,
                          const uint32_t tick)
{
  // TODO: update observation variables


  // Run inference
  if (runInference())
  {
    // Map nn output to control commands
    for (int i = 0; i < 4; i++)
    {
      control->thrust[i] = action_output[i];
    }
  }
  else
  {
    LOG_ERROR("Skipping control update due to inference failure.");
  }
}

