/*
The MIT License (MIT)

Copyright (c) 2024 Khaled Wahba

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

/*
This controller is based on the following publication:

Taeyoung Lee, Melvin Leok, and N. Harris McClamroch
Geometric Tracking Control of a Quadrotor UAV on SE(3)
CDC 2010

* Difference to Mellinger:
  * Different angular velocity error
  * Higher-order terms in attitude controller
*/

#include <math.h>
#include <string.h>

#include "log.h"
#include "param.h"

#include "math3d.h"
#include "controller_lee.h"
#include "physicalConstants.h"
#include "power_distribution.h"
#include "platform_defaults.h"

#include "filter.h"
#include "usec_time.h"
#include "debug.h"

static controllerLee_t g_self = {
  .mass = CF_MASS,

  // Inertia matrix (diagonal matrix), see
  // System Identification of the Crazyflie 2.0 Nano Quadrocopter
  // BA theses, Julian Foerster, ETHZ
  // https://polybox.ethz.ch/index.php/s/20dde63ee00ffe7085964393a55a91c7
  .J = {16.571710e-6, 16.655602e-6, 29.261652e-6}, // kg m^2

  // Position PID
  .Kpos_P = {7.0, 7.0, 7.0}, // Kp in paper
  .Kpos_P_limit = 100,
  .Kpos_D = {4.0, 4.0, 4.0}, // Kv in paper
  .Kpos_D_limit = 100,
  .Kpos_I = {0.0, 0.0, 0.0}, // not in paper
  .Kpos_I_limit = 2,

  // Attitude PID
  .KR = {0.007, 0.007, 0.008},
  .Komega = {0.00115, 0.00115, 0.002},
  .KI = {0.03, 0.03, 0.03},

  // INDI
  .indi = 0,
};


static bool rpm_deck_available;
static logVarId_t logVarRpm1;
static logVarId_t logVarRpm2;
static logVarId_t logVarRpm3;
static logVarId_t logVarRpm4;

static Butterworth2LowPass filter_acc_rpm[3];
static Butterworth2LowPass filter_acc_imu[3];
static Butterworth2LowPass filter_tau_rpm[3];
static Butterworth2LowPass filter_angular_acc[3];

static inline struct vec vclampscl(struct vec value, float min, float max) {
  return mkvec(
    clamp(value.x, min, max),
    clamp(value.y, min, max),
    clamp(value.z, min, max));
}

static inline void update_butterworth_2_low_pass_vec(Butterworth2LowPass filter[3], struct vec value) {
  update_butterworth_2_low_pass(&filter[0], value.x);
  update_butterworth_2_low_pass(&filter[1], value.y);
  update_butterworth_2_low_pass(&filter[2], value.z);
}

static inline struct vec get_butterworth_2_low_pass_vec(Butterworth2LowPass filter[3]) {
  float x = get_butterworth_2_low_pass(&filter[0]);
  float y = get_butterworth_2_low_pass(&filter[1]);
  float z = get_butterworth_2_low_pass(&filter[2]);
  return mkvec(x, y, z);
}

void controllerLeeReset(controllerLee_t* self)
{
  self->i_error_pos = vzero();
  self->i_error_att = vzero();
}

void controllerLeeInit(controllerLee_t* self)
{
  // copy default values (bindings), or NOP (firmware)
  *self = g_self;

  paramVarId_t idDeckBcRpm = paramGetVarId("deck", "bcRpm");
  logVarRpm1 = logGetVarId("rpm", "m1");
  logVarRpm2 = logGetVarId("rpm", "m2");
  logVarRpm3 = logGetVarId("rpm", "m3");
  logVarRpm4 = logGetVarId("rpm", "m4");

  rpm_deck_available = (paramGetUint(idDeckBcRpm) == 1);

	for (int8_t i = 0; i < 3; i++) {
    const float cutoff = 30; // Hz
		init_butterworth_2_low_pass(&filter_acc_rpm[i], 1 / (2 * M_PI_F * cutoff), 1.0 / ATTITUDE_RATE, 0.0f);
		init_butterworth_2_low_pass(&filter_acc_imu[i], 1 / (2 * M_PI_F * cutoff), 1.0 / ATTITUDE_RATE, 0.0f);

		init_butterworth_2_low_pass(&filter_tau_rpm[i], 1 / (2 * M_PI_F * cutoff), 1.0 / ATTITUDE_RATE, 0.0f);
		init_butterworth_2_low_pass(&filter_angular_acc[i], 1 / (2 * M_PI_F * cutoff), 1.0 / ATTITUDE_RATE, 0.0f);
	}

  self->timestamp_prev = usecTimestamp();
  self->omega_prev = vzero();

  controllerLeeReset(self);
}

bool controllerLeeTest(controllerLee_t* self)
{
  return true;
}

void controllerLee(controllerLee_t* self, control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{

  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }

  // uint64_t startTime = usecTimestamp();

  float dt = (float)(1.0f/ATTITUDE_RATE);
  // struct vec dessnap = vzero();
  // Address inconsistency in firmware where we need to compute our own desired yaw angle
  // Rate-controlled YAW is moving YAW angle setpoint
  float desiredYaw = 0; //rad
  if (setpoint->mode.yaw == modeVelocity) {
    desiredYaw = radians(state->attitude.yaw + setpoint->attitudeRate.yaw * dt);
  } else if (setpoint->mode.yaw == modeAbs) {
    desiredYaw = radians(setpoint->attitude.yaw);
  } else if (setpoint->mode.quat == modeAbs) {
    struct quat setpoint_quat = mkquat(setpoint->attitudeQuaternion.x, setpoint->attitudeQuaternion.y, setpoint->attitudeQuaternion.z, setpoint->attitudeQuaternion.w);
    self->rpy_des = quat2rpy(setpoint_quat);
    desiredYaw = self->rpy_des.z;
  }

  // INDI
  float t1 = 0.0f, t2 = 0.0f, t3 = 0.0f, t4 = 0.0f;
  if (self->indi && rpm_deck_available) {
    // compute expected acceleration based on rpm measurements (world frame)
    uint16_t rpm1 = logGetUint(logVarRpm1);
    uint16_t rpm2 = logGetUint(logVarRpm2);
    uint16_t rpm3 = logGetUint(logVarRpm3);
    uint16_t rpm4 = logGetUint(logVarRpm4);

    const float kappa_f = 2.6835255e-10f;
    t1 = kappa_f * powf(rpm1, 2);
    t2 = kappa_f * powf(rpm2, 2);
    t3 = kappa_f * powf(rpm3, 2);
    t4 = kappa_f * powf(rpm4, 2);

    // DEBUG
    if (tick % 500 == 0) {
      
      DEBUG_PRINT("INDI t %f %f %f %f\n", (double)t1, (double)t2, (double)t3, (double)t4);
    }
  }

  // Position controller
  if (   setpoint->mode.x == modeAbs
      || setpoint->mode.y == modeAbs
      || setpoint->mode.z == modeAbs) {
    struct vec pos_d = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
    struct vec vel_d = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
    struct vec acc_d = mkvec(setpoint->acceleration.x, setpoint->acceleration.y, setpoint->acceleration.z + GRAVITY_MAGNITUDE);
    struct vec statePos = mkvec(state->position.x, state->position.y, state->position.z);
    struct vec stateVel = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);

    // errors
    struct vec pos_e = vclampscl(vsub(pos_d, statePos), -self->Kpos_P_limit, self->Kpos_P_limit);
    struct vec vel_e = vclampscl(vsub(vel_d, stateVel), -self->Kpos_D_limit, self->Kpos_D_limit);
    self->i_error_pos = vadd(self->i_error_pos, vscl(dt, pos_e));
    self->p_error = pos_e;
    self->v_error = vel_e;

    // desired acceleration
    struct vec a_d = vadd4(
      acc_d,
      veltmul(self->Kpos_D, vel_e),
      veltmul(self->Kpos_P, pos_e),
      veltmul(self->Kpos_I, self->i_error_pos));

    struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
    struct mat33 R = quat2rotmat(q);
    struct vec z  = vbasis(2);

    // INDI
    struct vec a_indi = vzero();
    if ((self->indi & 1) && rpm_deck_available) {

      float f_rpm = t1 + t2 + t3 + t4;
      struct vec a_rpm = vsub(vscl(f_rpm / self->mass, mvmul(R, z)), mkvec(0.0, 0.0, 9.81f));
      update_butterworth_2_low_pass_vec(filter_acc_rpm, a_rpm);

      // compute acceleration based on IMU (world frame, SI unit, no gravity)
      struct vec a_imu = vscl(9.81, mkvec(state->acc.x, state->acc.y, state->acc.z));
      update_butterworth_2_low_pass_vec(filter_acc_imu, a_imu);

      struct vec a_rpm_filtered = get_butterworth_2_low_pass_vec(filter_acc_rpm);
      struct vec a_imu_filtered = get_butterworth_2_low_pass_vec(filter_acc_imu);

      a_indi = vsub(a_rpm_filtered, a_imu_filtered);

      // DEBUG
      if (tick % 500 == 0) {
        DEBUG_PRINT("INDI p %f %f %f, %f %f %f\n", (double)a_rpm_filtered.x, (double)a_rpm_filtered.y, (double)a_rpm_filtered.z, (double)a_imu_filtered.x, (double)a_imu_filtered.y, (double)a_imu_filtered.z);
      }
    }

    control->thrustSi = self->mass*vdot(vadd(a_d, a_indi), mvmul(R, z));

    self->thrustSi = control->thrustSi;
    // Reset the accumulated error while on the ground
    if (control->thrustSi < 0.01f) {
      controllerLeeReset(self);
    }

    // Compute Desired Rotation matrix
    float normFd = control->thrustSi;

    struct vec xdes = vbasis(0);
    struct vec ydes = vbasis(1);
    struct vec zdes = vbasis(2);
   
    if (normFd > 0) {
      zdes = vnormalize(vadd(a_d, a_indi));
    } 
    struct vec xcdes = mkvec(cosf(desiredYaw), sinf(desiredYaw), 0); 
    struct vec zcrossx = vcross(zdes, xcdes);
    float normZX = vmag(zcrossx);

    if (normZX > 0) {
      ydes = vnormalize(zcrossx);
    } 
    xdes = vcross(ydes, zdes);
    
    self->R_des = mcolumns(xdes, ydes, zdes);

  } else {
    if (setpoint->mode.z == modeDisable) {
      if (setpoint->thrust < 1000) {
          control->controlMode = controlModeForceTorque;
          control->thrustSi  = 0;
          control->torque[0] = 0;
          control->torque[1] = 0;
          control->torque[2] = 0;
          controllerLeeReset(self);
          return;
      }
    }
    const float max_thrust = powerDistributionGetMaxThrust(); // N
    control->thrustSi = setpoint->thrust / UINT16_MAX * max_thrust;

    struct quat q = rpy2quat(mkvec(
        radians(setpoint->attitude.roll),
        -radians(setpoint->attitude.pitch), // This is in the legacy coordinate system where pitch is inverted
        desiredYaw));
    self->R_des = quat2rotmat(q);
  }

  // Attitude controller

  // current rotation [R]
  struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
  self->rpy = quat2rpy(q);
  struct mat33 R = quat2rotmat(q);

  // desired rotation [Rdes]
  struct quat q_des = mat2quat(self->R_des);
  self->rpy_des = quat2rpy(q_des);

  // rotation error
  struct mat33 eRM = msub(mmul(mtranspose(self->R_des), R), mmul(mtranspose(R), self->R_des));

  struct vec eR = vscl(0.5f, mkvec(eRM.m[2][1], eRM.m[0][2], eRM.m[1][0]));

  // angular velocity
  self->omega = mkvec(
    radians(sensors->gyro.x),
    radians(sensors->gyro.y),
    radians(sensors->gyro.z));

  // Compute desired omega
  struct vec xdes = mcolumn(self->R_des, 0);
  struct vec ydes = mcolumn(self->R_des, 1);
  struct vec zdes = mcolumn(self->R_des, 2);
  struct vec hw = vzero();
  // Desired Jerk and snap for now are zeros vector
  struct vec desJerk = mkvec(setpoint->jerk.x, setpoint->jerk.y, setpoint->jerk.z);

  if (control->thrustSi != 0) {
    struct vec tmp = vsub(desJerk, vscl(vdot(zdes, desJerk), zdes));
    hw = vscl(self->mass/control->thrustSi, tmp);
  }
  struct vec z_w = mkvec(0,0,1); 
  float desiredYawRate = radians(setpoint->attitudeRate.yaw) * vdot(zdes,z_w);
  struct vec omega_des = mkvec(-vdot(hw,ydes), vdot(hw,xdes), desiredYawRate);
  
  self->omega_r = mvmul(mmul(mtranspose(R), self->R_des), omega_des);

  struct vec omega_error = vsub(self->omega, self->omega_r);
  
  // Integral part on angle
  self->i_error_att = vadd(self->i_error_att, vscl(dt, eR));

  // compute moments
  // M = -kR eR - kw ew + w x Jw - J(w x wr)
  self->u = vadd4(
    vneg(veltmul(self->KR, eR)),
    vneg(veltmul(self->Komega, omega_error)),
    vneg(veltmul(self->KI, self->i_error_att)),
    vcross(self->omega, veltmul(self->J, self->omega)));

  struct vec indi_moments;
  if ((self->indi & 2) && rpm_deck_available) {
    const float t2t = 0.006f;
    const float arm = 0.707106781f * 0.046f;
    struct vec tau_rpm = mkvec(
      -arm * t1 - arm * t2 + arm * t3 + arm * t4,
      -arm * t1 + arm * t2 + arm * t3 - arm * t4,
      -t2t * t1 + t2t * t2 - t2t * t3 + t2t * t4
    );
    update_butterworth_2_low_pass_vec(filter_tau_rpm, tau_rpm);

    struct vec tau_rpm_filtered = get_butterworth_2_low_pass_vec(filter_tau_rpm);

    // angular accelleration
    uint64_t timestamp = usecTimestamp();
    float dt = (timestamp - self->timestamp_prev) / 1e6;
    struct vec angular_acc = vdiv(vsub(self->omega, self->omega_prev), dt);
    update_butterworth_2_low_pass_vec(filter_angular_acc, angular_acc);

    struct vec angular_acc_filtered = get_butterworth_2_low_pass_vec(filter_angular_acc);
    struct vec tau_gyro_filtered = veltmul(self->J, angular_acc_filtered);

    self->omega_prev = self->omega;
    self->timestamp_prev = timestamp;

    indi_moments = vsub(tau_rpm_filtered, tau_gyro_filtered);
    self->u = vadd(self->u, indi_moments);

    // DEBUG
    if (tick % 1000 == 0) {
      DEBUG_PRINT("INDI a %f %f %f, %f %f %f\n", (double)tau_rpm_filtered.x, (double)tau_rpm_filtered.y, (double)tau_rpm_filtered.z, (double)tau_gyro_filtered.x, (double)tau_gyro_filtered.y, (double)tau_gyro_filtered.z);
    }
  }

  control->controlMode = controlModeForceTorque;
  control->torque[0] = self->u.x;
  control->torque[1] = self->u.y;
  control->torque[2] = self->u.z;

  // ticks = usecTimestamp() - startTime;
}

#ifdef CRAZYFLIE_FW

#include "param.h"
#include "log.h"

void controllerLeeFirmwareInit(void)
{
  controllerLeeInit(&g_self);
}

bool controllerLeeFirmwareTest(void)
{
  return true;
}

void controllerLeeFirmware(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  controllerLee(&g_self, control, setpoint, sensors, state, tick);
}

PARAM_GROUP_START(ctrlLee)
PARAM_ADD(PARAM_FLOAT, KR_x, &g_self.KR.x)
PARAM_ADD(PARAM_FLOAT, KR_y, &g_self.KR.y)
PARAM_ADD(PARAM_FLOAT, KR_z, &g_self.KR.z)
// Attitude I
PARAM_ADD(PARAM_FLOAT, KI_x, &g_self.KI.x)
PARAM_ADD(PARAM_FLOAT, KI_y, &g_self.KI.y)
PARAM_ADD(PARAM_FLOAT, KI_z, &g_self.KI.z)
// Attitude D
PARAM_ADD(PARAM_FLOAT, Kw_x, &g_self.Komega.x)
PARAM_ADD(PARAM_FLOAT, Kw_y, &g_self.Komega.y)
PARAM_ADD(PARAM_FLOAT, Kw_z, &g_self.Komega.z)

// J
PARAM_ADD(PARAM_FLOAT, J_x, &g_self.J.x)
PARAM_ADD(PARAM_FLOAT, J_y, &g_self.J.y)
PARAM_ADD(PARAM_FLOAT, J_z, &g_self.J.z)

// Position P
PARAM_ADD(PARAM_FLOAT, Kpos_Px, &g_self.Kpos_P.x)
PARAM_ADD(PARAM_FLOAT, Kpos_Py, &g_self.Kpos_P.y)
PARAM_ADD(PARAM_FLOAT, Kpos_Pz, &g_self.Kpos_P.z)
PARAM_ADD(PARAM_FLOAT, Kpos_P_limit, &g_self.Kpos_P_limit)
// Position D
PARAM_ADD(PARAM_FLOAT, Kpos_Dx, &g_self.Kpos_D.x)
PARAM_ADD(PARAM_FLOAT, Kpos_Dy, &g_self.Kpos_D.y)
PARAM_ADD(PARAM_FLOAT, Kpos_Dz, &g_self.Kpos_D.z)
PARAM_ADD(PARAM_FLOAT, Kpos_D_limit, &g_self.Kpos_D_limit)
// Position I
PARAM_ADD(PARAM_FLOAT, Kpos_Ix, &g_self.Kpos_I.x)
PARAM_ADD(PARAM_FLOAT, Kpos_Iy, &g_self.Kpos_I.y)
PARAM_ADD(PARAM_FLOAT, Kpos_Iz, &g_self.Kpos_I.z)
PARAM_ADD(PARAM_FLOAT, Kpos_I_limit, &g_self.Kpos_I_limit)

PARAM_ADD(PARAM_FLOAT, mass, &g_self.mass)

PARAM_ADD(PARAM_UINT8, indi, &g_self.indi)
PARAM_GROUP_STOP(ctrlLee)


LOG_GROUP_START(ctrlLee)

LOG_ADD(LOG_FLOAT, KR_x, &g_self.KR.x)
LOG_ADD(LOG_FLOAT, KR_y, &g_self.KR.y)
LOG_ADD(LOG_FLOAT, KR_z, &g_self.KR.z)
LOG_ADD(LOG_FLOAT, Kw_x, &g_self.Komega.x)
LOG_ADD(LOG_FLOAT, Kw_y, &g_self.Komega.y)
LOG_ADD(LOG_FLOAT, Kw_z, &g_self.Komega.z)

LOG_ADD(LOG_FLOAT,Kpos_Px, &g_self.Kpos_P.x)
LOG_ADD(LOG_FLOAT,Kpos_Py, &g_self.Kpos_P.y)
LOG_ADD(LOG_FLOAT,Kpos_Pz, &g_self.Kpos_P.z)
LOG_ADD(LOG_FLOAT,Kpos_Dx, &g_self.Kpos_D.x)
LOG_ADD(LOG_FLOAT,Kpos_Dy, &g_self.Kpos_D.y)
LOG_ADD(LOG_FLOAT,Kpos_Dz, &g_self.Kpos_D.z)


LOG_ADD(LOG_FLOAT, thrustSi, &g_self.thrustSi)
LOG_ADD(LOG_FLOAT, torquex, &g_self.u.x)
LOG_ADD(LOG_FLOAT, torquey, &g_self.u.y)
LOG_ADD(LOG_FLOAT, torquez, &g_self.u.z)

// current angles
LOG_ADD(LOG_FLOAT, rpyx, &g_self.rpy.x)
LOG_ADD(LOG_FLOAT, rpyy, &g_self.rpy.y)
LOG_ADD(LOG_FLOAT, rpyz, &g_self.rpy.z)

// desired angles
LOG_ADD(LOG_FLOAT, rpydx, &g_self.rpy_des.x)
LOG_ADD(LOG_FLOAT, rpydy, &g_self.rpy_des.y)
LOG_ADD(LOG_FLOAT, rpydz, &g_self.rpy_des.z)

// errors
LOG_ADD(LOG_FLOAT, error_posx, &g_self.p_error.x)
LOG_ADD(LOG_FLOAT, error_posy, &g_self.p_error.y)
LOG_ADD(LOG_FLOAT, error_posz, &g_self.p_error.z)

LOG_ADD(LOG_FLOAT, error_velx, &g_self.v_error.x)
LOG_ADD(LOG_FLOAT, error_vely, &g_self.v_error.y)
LOG_ADD(LOG_FLOAT, error_velz, &g_self.v_error.z)

// omega
LOG_ADD(LOG_FLOAT, omegax, &g_self.omega.x)
LOG_ADD(LOG_FLOAT, omegay, &g_self.omega.y)
LOG_ADD(LOG_FLOAT, omegaz, &g_self.omega.z)

// omega_r
LOG_ADD(LOG_FLOAT, omegarx, &g_self.omega_r.x)
LOG_ADD(LOG_FLOAT, omegary, &g_self.omega_r.y)
LOG_ADD(LOG_FLOAT, omegarz, &g_self.omega_r.z)

// LOG_ADD(LOG_UINT32, ticks, &ticks)

LOG_GROUP_STOP(ctrlLee)

#endif // CRAZYFLIE_FW defined
