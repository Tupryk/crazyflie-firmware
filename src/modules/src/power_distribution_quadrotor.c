/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2022 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * power_distribution_quadrotor.c - Crazyflie stock power distribution code
 */

#include "power_distribution.h"

#include <string.h>
#include "log.h"
#include "param.h"
#include "num.h"
#include "autoconf.h"
#include "config.h"
#include "math.h"
#include "math3d.h"

#ifndef CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#  define DEFAULT_IDLE_THRUST 0
#else
#  define DEFAULT_IDLE_THRUST CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#endif

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;
static float armLength = 0.046f; // m;
static float thrustToTorque = 0.005964552f;

// thrust = a * pwm^2 + b * pwm
static float pwmToThrustA = 0.091492681f;
static float pwmToThrustB = 0.067673604f;

int powerDistributionMotorType(uint32_t id)
{
  return 1;
}

uint16_t powerDistributionStopRatio(uint32_t id)
{
  return 0;
}

void powerDistributionInit(void)
{
}

bool powerDistributionTest(void)
{
  bool pass = true;
  return pass;
}

static uint16_t capMinThrust(float thrust, uint32_t minThrust) {
  if (thrust < minThrust) {
    return minThrust;
  }

  return thrust;
}

static void powerDistributionLegacy(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped)
{
  int16_t r = control->roll / 2.0f;
  int16_t p = control->pitch / 2.0f;

  motorThrustUncapped->motors.m1 = control->thrust - r + p + control->yaw;
  motorThrustUncapped->motors.m2 = control->thrust - r - p - control->yaw;
  motorThrustUncapped->motors.m3 = control->thrust + r - p + control->yaw;
  motorThrustUncapped->motors.m4 = control->thrust + r + p - control->yaw;
}

static void powerDistributionForceTorque(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped) {
  static float motorForces[STABILIZER_NR_OF_MOTORS];

  const float arm = 0.707106781f * armLength;
  const float rollPart = 0.25f / arm * control->torqueX;
  const float pitchPart = 0.25f / arm * control->torqueY;
  const float thrustPart = 0.25f * control->thrustSi; // N (per rotor)
  const float yawPart = 0.25f * control->torqueZ / thrustToTorque;

  motorForces[0] = thrustPart - rollPart - pitchPart - yawPart;
  motorForces[1] = thrustPart - rollPart + pitchPart + yawPart;
  motorForces[2] = thrustPart + rollPart + pitchPart - yawPart;
  motorForces[3] = thrustPart + rollPart - pitchPart + yawPart;

  for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++) {
    float motorForce = motorForces[motorIndex];
    if (motorForce < 0.0f) {
      motorForce = 0.0f;
    }

    float motor_pwm = (-pwmToThrustB + sqrtf(pwmToThrustB * pwmToThrustB + 4.0f * pwmToThrustA * motorForce)) / (2.0f * pwmToThrustA);
    motorThrustUncapped->list[motorIndex] = motor_pwm * UINT16_MAX;
  }
}

static void powerDistributionForce(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped) {
  // Not implemented yet
}

void powerDistribution(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped)
{
  switch (control->controlMode) {
    case controlModeLegacy:
      powerDistributionLegacy(control, motorThrustUncapped);
      break;
    case controlModeForceTorque:
      powerDistributionForceTorque(control, motorThrustUncapped);
      break;
    case controlModeForce:
      powerDistributionForce(control, motorThrustUncapped);
      break;
    default:
      // Nothing here
      break;
  }
}

void powerDistributionCap(const motors_thrust_uncapped_t* motorThrustBatCompUncapped, motors_thrust_pwm_t* motorPwm)
{
  const int32_t maxAllowedThrust = UINT16_MAX;

  // Find highest thrust
  int32_t highestThrustFound = 0;
  for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++)
  {
    const int32_t thrust = motorThrustBatCompUncapped->list[motorIndex];
    if (thrust > highestThrustFound)
    {
      highestThrustFound = thrust;
    }
  }

  int32_t reduction = 0;
  if (highestThrustFound > maxAllowedThrust)
  {
    reduction = highestThrustFound - maxAllowedThrust;
  }

  for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++)
  {
    int32_t thrustCappedUpper = motorThrustBatCompUncapped->list[motorIndex] - reduction;
    motorPwm->list[motorIndex] = capMinThrust(thrustCappedUpper, idleThrust);
  }
}

#define limitThrust(VAL) limitUint16(VAL)

static void powerDistributionLegacyOld(motors_thrust_t* motorPower, const control_t *control)
{
  motorPower->mode = motorsThrustModePWM;
  int16_t r = control->roll / 2.0f;
  int16_t p = control->pitch / 2.0f;
  motorPower->m1 = limitThrust(control->thrust - r + p + control->yaw);
  motorPower->m2 = limitThrust(control->thrust - r - p - control->yaw);
  motorPower->m3 =  limitThrust(control->thrust + r - p + control->yaw);
  motorPower->m4 =  limitThrust(control->thrust + r + p - control->yaw);

  if (motorPower->m1 < idleThrust) {
    motorPower->m1 = idleThrust;
  }
  if (motorPower->m2 < idleThrust) {
    motorPower->m2 = idleThrust;
  }
  if (motorPower->m3 < idleThrust) {
    motorPower->m3 = idleThrust;
  }
  if (motorPower->m4 < idleThrust) {
    motorPower->m4 = idleThrust;
  }
}

static void powerDistributionForceTorqueOld(motors_thrust_t* motorPower, const control_t *control, float maxThrust)
{
  // On CF2, thrust is mapped 65536 <==> 60 grams
  // float thrust = control->thrustSi;
  struct vec torque = mkvec(control->torque[0], control->torque[1], control->torque[2]);

  // torque.x = clamp(torque.x, -0.002, 0.002);
  // torque.y = clamp(torque.y, -0.002, 0.002);
  // torque.z = clamp(torque.z, -0.0005, 0.0005);

  const float thrust_to_torque = 0.006f;
  const float arm_length = 0.046f; // m

  // see https://github.com/jpreiss/libquadrotor/blob/master/src/quad_control.c
  const float thrustpart = 0.25f * control->thrustSi; // N (per rotor)
  const float yawpart = -0.25f * torque.z / thrust_to_torque;

  float const arm = 0.707106781f * arm_length;
  const float rollpart = 0.25f / arm * torque.x;
  const float pitchpart = 0.25f / arm * torque.y;

  // Simple thrust mixing
  float motorForce0 = thrustpart - rollpart - pitchpart + yawpart;
  float motorForce1 = thrustpart - rollpart + pitchpart - yawpart;
  float motorForce2 = thrustpart + rollpart + pitchpart + yawpart;
  float motorForce3 = thrustpart + rollpart - pitchpart - yawpart;

  // for CF2, motorratio directly maps to thrust (not rpm etc.)
  // Thus, we only need to scale the values here
  motorPower->mode = motorsThrustModeForce;
  motorPower->f1 = clamp(motorForce0 / 9.81f * 1000.0f, 0, maxThrust);
  motorPower->f2 = clamp(motorForce1 / 9.81f * 1000.0f, 0, maxThrust);
  motorPower->f3 = clamp(motorForce2 / 9.81f * 1000.0f, 0, maxThrust);
  motorPower->f4 = clamp(motorForce3 / 9.81f * 1000.0f, 0, maxThrust);
}

static void powerDistributionForceOld(motors_thrust_t* motorPower, const control_t *control, float maxThrust)
{
  motorPower->mode = motorsThrustModeForce;
  motorPower->f1 = control->normalizedForces[0] * maxThrust;
  motorPower->f2 = control->normalizedForces[1] * maxThrust;
  motorPower->f3 = control->normalizedForces[2] * maxThrust;
  motorPower->f4 = control->normalizedForces[3] * maxThrust;
}

void powerDistributionOld(motors_thrust_t* motorPower, const control_t *control, float maxThrust)
{
  switch (control->controlMode)
  {
    case controlModeLegacy:
      powerDistributionLegacyOld(motorPower, control);
      break;
    case controlModeForceTorque:
      powerDistributionForceTorqueOld(motorPower, control, maxThrust);
      break;
    case controlModeForce:
      powerDistributionForceOld(motorPower, control, maxThrust);
      break;
  }
}


/**
 * Power distribution parameters
 */
PARAM_GROUP_START(powerDist)
/**
 * @brief Motor thrust to set at idle (default: 0)
 *
 * This is often needed for brushless motors as
 * it takes time to start up the motor. Then a
 * common value is between 3000 - 6000.
 */
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_PERSISTENT, idleThrust, &idleThrust)
PARAM_GROUP_STOP(powerDist)

/**
 * System identification parameters for quad rotor
 */
PARAM_GROUP_START(quadSysId)

PARAM_ADD(PARAM_FLOAT, thrustToTorque, &thrustToTorque)
PARAM_ADD(PARAM_FLOAT, pwmToThrustA, &pwmToThrustA)
PARAM_ADD(PARAM_FLOAT, pwmToThrustB, &pwmToThrustB)

/**
 * @brief Length of arms (m)
 *
 * The distance from the center to a motor
 */
PARAM_ADD(PARAM_FLOAT, armLength, &armLength)
PARAM_GROUP_STOP(quadSysId)
