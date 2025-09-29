#include <math.h>
#include <string.h>
#include <inttypes.h>
#include <stdio.h>

#include "math3d.h"
#include "controller_lee_payload.h"
#include "physicalConstants.h"
#include "power_distribution.h"
#include "platform_defaults.h"
#include "param.h"
#include "log.h"
#include "debug.h"
#include "usec_time.h"
#include "motors.h"


static controllerLeePayload_t g_self = {
  .mass = 0.034,
  .mp   = 0.01,
  // Inertia matrix (diagonal matrix), see
  // System Identification of the Crazyflie 2.0 Nano Quadrocopter
  // BA theses, Julian Foerster, ETHZ
  // https://polybox.ethz.ch/index.php/s/20dde63ee00ffe7085964393a55a91c7
  .J = {16.571710e-6, 16.655602e-6, 29.261652e-6}, // kg m^2

  // Payload PID
  .Kpos_P = {7.0, 7.0, 7.0}, // Kp in paper
  .Kpos_P_limit = 100,
  .Kpos_D = {4.0, 4.0, 4.0}, // Kv in paper
  .Kpos_D_limit = 100,
  .Kpos_I = {0.0, 0.0, 0.0}, // not in paper
  .Kpos_I_limit = 100,

  // Cable PD 
  .K_q = {1.0, 1.0, 1.0}, // cable direction P
  .K_q_limit = 100,
  .K_w = {0.1, 0.1, 0.1}, // cable angular velocity D
  .K_w_limit = 100,
  .K_q_I = {0.0, 0.0, 0.0}, // cable direction I

  // UAV Position PID Gains
  .Kpos_UAV_P = {7.0, 7.0, 7.0}, // UAV position Kp
  .Kpos_UAV_P_limit = 100,
  .Kpos_UAV_D = {4.0, 4.0, 4.0}, // UAV position Kv
  .Kpos_UAV_D_limit = 100,
  .Kpos_UAV_I = {0.0, 0.0, 0.0}, // UAV position Ki
  .Kpos_UAV_I_limit = 100,

  // UAV Attitude PID
  .KR = {0.007, 0.007, 0.008},
  .Komega = {0.00115, 0.00115, 0.002},
  .KI = {0.03, 0.03, 0.03},

  .attachement_points[0].l = -1,
  .attachement_points[1].l = -1,
  .attachement_points[2].l = -1,


};

static inline struct vec vclampscl(struct vec value, float min, float max) {
  return mkvec(
    clamp(value.x, min, max),
    clamp(value.y, min, max),
    clamp(value.z, min, max));
}

void controllerLeePayloadReset(controllerLeePayload_t* self)
{
  self->i_error_pos = vzero();
  self->i_error_q = vzero();
  self->i_error_pos_uav = vzero();
  self->i_error_att_uav = vzero();
}


void controllerLeePayloadInit(controllerLeePayload_t* self)
{
  // copy default values (bindings), or NOP (firmware)
  *self = g_self;

  controllerLeePayloadReset(self);
}

void controllerLeePayload(controllerLeePayload_t* self, control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{

  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }
  float dt = (float)(1.0f/ATTITUDE_RATE);
  float desiredYaw = 0; //rad
  
  if (setpoint->mode.yaw == modeVelocity) {
    desiredYaw = radians(state->attitude.yaw + setpoint->attitudeRate.yaw * dt);
  } else if (setpoint->mode.yaw == modeAbs) {
    desiredYaw = radians(setpoint->attitude.yaw);
  }
  
  struct vec xc = mkvec(cosf(desiredYaw), sinf(desiredYaw), 0);
  struct vec yc = mkvec(-sinf(desiredYaw), cosf(desiredYaw), 0);


  // Position controller
  if (   setpoint->mode.x == modeAbs
      || setpoint->mode.y == modeAbs
      || setpoint->mode.z == modeAbs) {
    struct vec gravity_comp = mkvec(0, 0, GRAVITY_MAGNITUDE);
    // payload desired trajectory (from setpoint)
    struct vec plPos_d = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
    struct vec plVel_d = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
    struct vec plAcc_d = mkvec(setpoint->acceleration.x, setpoint->acceleration.y, setpoint->acceleration.z);
    struct vec plJerk_d = mkvec(setpoint->jerk.x, setpoint->jerk.y, setpoint->jerk.z);
    struct vec plSnap_d = mkvec(setpoint->snap.x, setpoint->snap.y, setpoint->snap.z);
    struct vec pldSnap_d = vzero();   // set to zero for now
    struct vec plddSnap_d = vzero(); // set to zero for now

    // payload position and velocity states
    struct vec plPos = mkvec(state->payload_pos.x, state->payload_pos.y, state->payload_pos.z);
    struct vec plVel = mkvec(state->payload_vel.x, state->payload_vel.y, state->payload_vel.z);
    struct vec plAcc = mkvec(state->payload_acc.x, state->payload_acc.y, state->payload_acc.z);
    
    //logging payload state
    self->pl_pos = plPos;
    self->pl_vel = plVel;
    self->pl_acc = plAcc;

    // UAV position and velocity states
    struct vec statePos = mkvec(state->position.x, state->position.y, state->position.z);
    struct vec stateVel = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);
        
    // position and velocity errors
    struct vec plpos_e = vclampscl(vsub(plPos_d, plPos), -self->Kpos_P_limit, self->Kpos_P_limit);
    struct vec plvel_e = vclampscl(vsub(plVel_d, plVel), -self->Kpos_D_limit, self->Kpos_D_limit);
    self->i_error_pos = vadd(self->i_error_pos, vscl(dt, plpos_e));

    struct vec plAcc_w_gcomp = vadd(plAcc, gravity_comp);
    // payload desired force
    self->F_d = vscl(self->mp, 
      vadd4(
      plAcc_w_gcomp,
      veltmul(self->Kpos_P, plpos_e),
      veltmul(self->Kpos_D, plvel_e),
      veltmul(self->Kpos_I, self->i_error_pos)
    ));

    // cable desired force (desired virtual input), assuming only 1 uav connected to payload
    self->desVirtInp.x = self->F_d.x;
    self->desVirtInp.y = self->F_d.y;
    self->desVirtInp.z = self->F_d.z;

    //directional unit vector qi and its derivative qidot from UAV to payload
    self->qi = vnormalize(vsub(plPos, statePos));
    self->qidot = vnormalize(vsub(plVel, stateVel));
    self->omega_c = vcross(self->qi, self->qidot); // cable angular velocity

    struct mat33 qiqiT = vecmult(self->qi);
    // projection of the desired virtual input along qi
    struct vec virtualInp = mvmul(qiqiT, self->desVirtInp);
    

    struct vec attPoint = mkvec(0, 0, 0);
    float l = -1;
    // find the attachment point for this UAV (the one, which doesn't have any neighbor associated with it)
    for (uint8_t i = 0; i < state->num_uavs; ++i) {
      if (self->attachement_points[i].id == state->team_state[0].id) {
        // this attachement point belongs to a neighbor

        attPoint = self->attachement_points[i].point;
        l = self->attachement_points[i].l;
        break;
      }
    }
    
    struct vec u_parallel = vadd3(virtualInp, vscl(self->mass*l*vmag2(self->omega_c), self->qi), vscl(self->mass, mvmul(qiqiT, plAcc_w_gcomp)));  

    // desired cable direction and its derivative
    self->qdi = vnormalize(self->desVirtInp);
    // eq. 68-71 in Aggressive Maneuvering of a Quadrotor with a Cable-Suspended Payload by Sarah Tang
    float T = self->mp*vmag(plAcc_w_gcomp); // cable tension
    float T_dot = -self->mp * vdot(plJerk_d, self->qi); // tension derivative
    self->qdidot = vdiv(vneg(vadd(vscl(self->mp, plJerk_d), vscl(T_dot, self->qdi))), T);
    self->omega_cd = vcross(vdiv(vscl(self->mp, plJerk_d), T), self->qi); // angular velocity of the cable, w in paper

    struct mat33 skewqi = mcrossmat(self->qi); // skew symmetric matrix of qi
    struct mat33 skewqi2 = mmul(skewqi,skewqi); // skewqi squared
    
    
    struct vec eq  = vclampscl(vcross(self->qdi, self->qi), -self->K_q_limit, self->K_q_limit);
    self->i_error_q = vadd(self->i_error_q, vscl(dt, eq));
    
    struct vec ew  = vclampscl(vadd(self->omega_c, mvmul(skewqi2, self->omega_cd)), -self->K_w_limit, self->K_w_limit);
    
    struct vec u_perpind = vsub(
      vscl(self->mass*l, 
        mvmul(skewqi,
          vadd4(
            vneg(veltmul(self->K_q, eq)),
            vneg(veltmul(self->K_w, ew)),
            vneg(vscl(vdot(self->qi, self->omega_cd), self->qidot)), 
            vneg(veltmul(self->K_q_I, self->i_error_q)) // not originally in the paper
          )
        )
      ),
      vscl(self->mass, mvmul(skewqi2, plAcc_w_gcomp))
    );

    struct vec u = vadd(u_parallel, u_perpind); // total desired force by the UAV on the cable
    //------------------------------------------------------------------------------------------//
    
    // UAV Lee controller
    
    struct vec pos_d = vsub(plPos, vscl(l, self->qi)); // desired UAV position
    struct vec vel_d = vsub(plVel, vscl(l, self->qidot)); // desired UAV velocity
    struct vec acc_d = vdiv(u, self->mass); // desired UAV acceleration

    struct vec pos_e = vclampscl(vsub(pos_d, statePos), -self->Kpos_UAV_P_limit, self->Kpos_UAV_P_limit);
    struct vec vel_e = vclampscl(vsub(vel_d, stateVel), -self->Kpos_UAV_D_limit, self->Kpos_UAV_D_limit);

    self->i_error_pos_uav = vadd(self->i_error_pos_uav, vscl(dt, pos_e));
    struct vec F_uav = vadd4(
      acc_d,
      veltmul(self->Kpos_UAV_P, pos_e),
      veltmul(self->Kpos_UAV_D, vel_e),
      veltmul(self->Kpos_UAV_I, self->i_error_pos_uav)
    );

    struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
    struct mat33 R = quat2rotmat(q);
    struct vec z  = vbasis(2);
    control->thrustSi = self->mass*vdot(F_uav , mvmul(R, z));
    self->thrustSi = control->thrustSi;
    
    if (control->thrustSi < 0.01f) {
      controllerLeePayloadReset(self);
    }
  
    struct vec xb = vnormalize(vcross(yc, F_uav));
    struct vec yb = vnormalize(vcross(F_uav, xb));
    struct vec zb = vcross(xb, yb);
    self->R_des = mcolumns(xb, yb, zb); // desired rotation matrix of the UAV

  } else {
    if (setpoint->mode.z == modeDisable) {
      if (setpoint->thrust < 1000) {
          control->controlMode = controlModeForceTorque;
          control->thrustSi  = 0;
          control->torque[0] = 0;
          control->torque[1] = 0;
          control->torque[2] = 0;
          controllerLeePayloadReset(self);
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

  // Lee attitude controller
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

  struct vec desJerk = vzero(); // desired UAV jerk set to zero for now
  struct vec desSnap = vzero(); // desired UAV snap set to zero for now


  // Compute desired omega
  struct vec xb = mcolumn(self->R_des, 0);
  struct vec yb = mcolumn(self->R_des, 1);
  struct vec zb = mcolumn(self->R_des, 2);

  float c = control->thrustSi / self->mass;
  float B1 = c;
  float B3 = -vdot(yc, zb);
  float C3 = vmag(vcross(yc, zb));
  float D1 = vdot(xb, desJerk);
  float D2 = -vdot(yb, desJerk);
  float D3 = radians(setpoint->attitudeRate.yaw) * vdot(xc, xb);
  
  struct vec omega_des = vzero();
  if (control->thrustSi != 0) {
    omega_des.x = D2/B1;
    omega_des.y = D1/B1;
    omega_des.z = (B1*D3-B3*D1)/(B1*C3);
  }

  // Compute desired omega dot
  float setpoint_yaw_ddot = radians(setpoint->attitudeAcc.yaw);
  float setpoint_yaw_dot = radians(setpoint->attitudeRate.yaw);

  float c_dot = vdot(zb, desJerk);
  float E1 = vdot(xb,desSnap) - 2.0f * c_dot * omega_des.y - c * omega_des.x * omega_des.z;
  float E2 = -vdot(yb,desSnap) - 2.0f * c_dot * omega_des.x + c * omega_des.y * omega_des.z;
  float E3 = setpoint_yaw_ddot * vdot(xc, xb) + 2.0f * setpoint_yaw_dot * omega_des.z * vdot(xc, yb) - 2.0f * setpoint_yaw_dot*omega_des.y*vdot(xc,zb) - omega_des.x*omega_des.y*vdot(yc,yb) - omega_des.x*omega_des.z*vdot(yc,zb);

  self->omega_des_dot = vzero();
  if (control->thrustSi != 0) {
    self->omega_des_dot.x = E2/B1;
    self->omega_des_dot.y = E1/B1;
    self->omega_des_dot.z = (B1*E3-B3*E1)/(B1*C3);
  }


  self->omega_r = mvmul(mmul(mtranspose(R), self->R_des), omega_des);

  struct vec omega_error = vsub(self->omega, self->omega_r);
  
  // Integral part on angle
  self->i_error_att_uav = vadd(self->i_error_att_uav, vscl(dt, eR));


  self->tau = vadd5(
    vneg(veltmul(self->KR, eR)),
    vneg(veltmul(self->Komega, omega_error)),
    vneg(veltmul(self->KI, self->i_error_att_uav)),
    vcross(self->omega, veltmul(self->J, self->omega)),
    vneg(veltmul(self->J, vsub(mvmul(mcrossmat(self->omega), self->omega_r), mvmul(mmul(mtranspose(R), self->R_des), self->omega_des_dot)))));

  control->controlMode = controlModeForceTorque;
  control->torque[0] = self->tau.x;
  control->torque[1] = self->tau.y;
  control->torque[2] = self->tau.z;

}




#ifdef CRAZYFLIE_FW

#include "param.h"
#include "log.h"

void controllerLeePayloadFirmwareInit(void)
{
  controllerLeePayloadInit(&g_self);
}

bool controllerLeePayloadFirmwareTest(void)
{
  return true;
}

void controllerLeePayloadFirmware(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  controllerLeePayload(&g_self, control, setpoint, sensors, state, tick);
}

PARAM_GROUP_START(ctrlLeeP)

// Payload Position P
PARAM_ADD(PARAM_FLOAT, Kpos_Px, &g_self.Kpos_P.x)
PARAM_ADD(PARAM_FLOAT, Kpos_Py, &g_self.Kpos_P.y)
PARAM_ADD(PARAM_FLOAT, Kpos_Pz, &g_self.Kpos_P.z)
PARAM_ADD(PARAM_FLOAT, Kpos_P_limit, &g_self.Kpos_P_limit)
// Payload Position D
PARAM_ADD(PARAM_FLOAT, Kpos_Dx, &g_self.Kpos_D.x)
PARAM_ADD(PARAM_FLOAT, Kpos_Dy, &g_self.Kpos_D.y)
PARAM_ADD(PARAM_FLOAT, Kpos_Dz, &g_self.Kpos_D.z)
PARAM_ADD(PARAM_FLOAT, Kpos_D_limit, &g_self.Kpos_D_limit)
// Payload Position I
PARAM_ADD(PARAM_FLOAT, Kpos_Ix, &g_self.Kpos_I.x)
PARAM_ADD(PARAM_FLOAT, Kpos_Iy, &g_self.Kpos_I.y)
PARAM_ADD(PARAM_FLOAT, Kpos_Iz, &g_self.Kpos_I.z)
PARAM_ADD(PARAM_FLOAT, Kpos_I_limit, &g_self.Kpos_I_limit)


// Cable P
PARAM_ADD(PARAM_FLOAT, Kqx, &g_self.K_q.x)
PARAM_ADD(PARAM_FLOAT, Kqy, &g_self.K_q.y)
PARAM_ADD(PARAM_FLOAT, Kqz, &g_self.K_q.z)
PARAM_ADD(PARAM_FLOAT, Kq_limit, &g_self.K_q_limit)

// Cable D
PARAM_ADD(PARAM_FLOAT, Kwx, &g_self.K_w.x)
PARAM_ADD(PARAM_FLOAT, Kwy, &g_self.K_w.y)
PARAM_ADD(PARAM_FLOAT, Kwz, &g_self.K_w.z)
PARAM_ADD(PARAM_FLOAT, Kw_limit, &g_self.K_w_limit)

// Cable I
PARAM_ADD(PARAM_FLOAT, KqIx, &g_self.K_q_I.x)
PARAM_ADD(PARAM_FLOAT, KqIy, &g_self.K_q_I.y)
PARAM_ADD(PARAM_FLOAT, KqIz, &g_self.K_q_I.z)

// UAV Position P
PARAM_ADD(PARAM_FLOAT, Kpos_UAV_Px, &g_self.Kpos_UAV_P.x)
PARAM_ADD(PARAM_FLOAT, Kpos_UAV_Py, &g_self.Kpos_UAV_P.y)
PARAM_ADD(PARAM_FLOAT, Kpos_UAV_Pz, &g_self.Kpos_UAV_P.z)
PARAM_ADD(PARAM_FLOAT, Kpos_UAV P_limit, &g_self.Kpos_UAV_P_limit)
// UAV Position D
PARAM_ADD(PARAM_FLOAT, Kpos_UAV_Dx, &g_self.Kpos_UAV_D.x)
PARAM_ADD(PARAM_FLOAT, Kpos_UAV_Dy, &g_self.Kpos_UAV_D.y)
PARAM_ADD(PARAM_FLOAT, Kpos_UAV_Dz, &g_self.Kpos_UAV_D.z)
PARAM_ADD(PARAM_FLOAT, Kpos_UAV_D_limit, &g_self.Kpos_UAV_D_limit)
// UAV Position I
PARAM_ADD(PARAM_FLOAT, Kpos_UAV_Ix, &g_self.Kpos_UAV_I.x)
PARAM_ADD(PARAM_FLOAT, Kpos_UAV_Iy, &g_self.Kpos_UAV_I.y)
PARAM_ADD(PARAM_FLOAT, Kpos_UAV_Iz, &g_self.Kpos_UAV_I.z)
PARAM_ADD(PARAM_FLOAT, Kpos_UAV_I_limit, &g_self.Kpos_UAV_I_limit)


// UAV Attitude P
PARAM_ADD(PARAM_FLOAT, KRx, &g_self.KR.x)
PARAM_ADD(PARAM_FLOAT, KRy, &g_self.KR.y)
PARAM_ADD(PARAM_FLOAT, KRz, &g_self.KR.z)
PARAM_ADD(PARAM_FLOAT, KR_limit, &g_self.KR_limit)

// UAV Attitude D
PARAM_ADD(PARAM_FLOAT, Komx, &g_self.Komega.x)
PARAM_ADD(PARAM_FLOAT, Komy, &g_self.Komega.y)
PARAM_ADD(PARAM_FLOAT, Komz, &g_self.Komega.z)
PARAM_ADD(PARAM_FLOAT, Kom_limit, &g_self.Komega_limit)

// UAV Attitude I
PARAM_ADD(PARAM_FLOAT, KI_x, &g_self.KI.x)
PARAM_ADD(PARAM_FLOAT, KI_y, &g_self.KI.y)
PARAM_ADD(PARAM_FLOAT, KI_z, &g_self.KI.z)


PARAM_ADD(PARAM_FLOAT, mass, &g_self.mass)
PARAM_ADD(PARAM_FLOAT, massP, &g_self.mp)


// Attachement points and cable lengths
PARAM_ADD(PARAM_UINT8, ap0id, &g_self.attachement_points[0].id)
PARAM_ADD(PARAM_FLOAT, ap0x, &g_self.attachement_points[0].point.x)
PARAM_ADD(PARAM_FLOAT, ap0y, &g_self.attachement_points[0].point.y)
PARAM_ADD(PARAM_FLOAT, ap0z, &g_self.attachement_points[0].point.z)
PARAM_ADD(PARAM_FLOAT, ap0l, &g_self.attachement_points[0].l)

PARAM_ADD(PARAM_UINT8, ap1id, &g_self.attachement_points[1].id)
PARAM_ADD(PARAM_FLOAT, ap1x, &g_self.attachement_points[1].point.x)
PARAM_ADD(PARAM_FLOAT, ap1y, &g_self.attachement_points[1].point.y)
PARAM_ADD(PARAM_FLOAT, ap1z, &g_self.attachement_points[1].point.z)
PARAM_ADD(PARAM_FLOAT, ap1l, &g_self.attachement_points[1].l)

PARAM_ADD(PARAM_UINT8, ap2id, &g_self.attachement_points[2].id)
PARAM_ADD(PARAM_FLOAT, ap2x, &g_self.attachement_points[2].point.x)
PARAM_ADD(PARAM_FLOAT, ap2y, &g_self.attachement_points[2].point.y)
PARAM_ADD(PARAM_FLOAT, ap2z, &g_self.attachement_points[2].point.z)
PARAM_ADD(PARAM_FLOAT, ap2l, &g_self.attachement_points[2].l)

PARAM_GROUP_STOP(ctrlLeeP)


LOG_GROUP_START(ctrlLeeP)

LOG_ADD(LOG_FLOAT, thrustSI, &g_self.thrustSi)
LOG_ADD(LOG_FLOAT, torquex, &g_self.tau.x)
LOG_ADD(LOG_FLOAT, torquey, &g_self.tau.y)
LOG_ADD(LOG_FLOAT, torquez, &g_self.tau.z)

// current angles
LOG_ADD(LOG_FLOAT, rpyx, &g_self.rpy.x)
LOG_ADD(LOG_FLOAT, rpyy, &g_self.rpy.y)
LOG_ADD(LOG_FLOAT, rpyz, &g_self.rpy.z)

// desired angles
LOG_ADD(LOG_FLOAT, rpydx, &g_self.rpy_des.x)
LOG_ADD(LOG_FLOAT, rpydy, &g_self.rpy_des.y)
LOG_ADD(LOG_FLOAT, rpydz, &g_self.rpy_des.z)


// omega
LOG_ADD(LOG_FLOAT, omegax, &g_self.omega.x)
LOG_ADD(LOG_FLOAT, omegay, &g_self.omega.y)
LOG_ADD(LOG_FLOAT, omegaz, &g_self.omega.z)

// omega_r
LOG_ADD(LOG_FLOAT, omegarx, &g_self.omega_r.x)
LOG_ADD(LOG_FLOAT, omegary, &g_self.omega_r.y)
LOG_ADD(LOG_FLOAT, omegarz, &g_self.omega_r.z)

LOG_ADD(LOG_FLOAT, plVelx, &g_self.pl_vel.x)
LOG_ADD(LOG_FLOAT, plVely, &g_self.pl_vel.y)
LOG_ADD(LOG_FLOAT, plVelz, &g_self.pl_vel.z)

LOG_ADD(LOG_FLOAT, plAccx, &g_self.pl_acc.x)
LOG_ADD(LOG_FLOAT, plAccy, &g_self.pl_acc.y)
LOG_ADD(LOG_FLOAT, plAccz, &g_self.pl_acc.z)
LOG_GROUP_STOP(ctrlLeeP)

#endif // CRAZYFLIE_FW defined
