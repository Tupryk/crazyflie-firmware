/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
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
 */
#ifndef __CONTROLLER_LEE_PAYLOAD_H__
#define __CONTROLLER_LEE_PAYLOAD_H__

#include "stabilizer_types.h"

// This structure contains the mutable state and inmutable parameters
typedef struct controllerLeePayload_s {
    float mass; // TODO: should be CF global for other modules
    float mp; // mass payload
    float thrustSI;
    struct vec J; // Inertia matrix (diagonal matrix); kg m^2
    struct vec offset; // offset for reference

    struct {
        uint8_t id;
        struct vec point;
        float l; // cable length; set to <= 0 to compute automatically based on the measurements
    } attachement_points[3];

    struct {
        uint8_t id1;
        uint8_t id2;
        struct mat66 Pinv;
    } Pinvs[3];
    
    //Position PID
    struct vec Kpos_P;
    float Kpos_P_limit;
    struct vec Kpos_D;
    float Kpos_D_limit;
    struct vec Kpos_I;
    float Kpos_I_limit;
    
    // Payload attitude control gains
    struct vec Kprot_P;
    float Kprot_P_limit;
    struct vec Kprot_D;
    float Kprot_D_limit;
    struct vec Kprot_I;
    float Kprot_I_limit;

    struct vec i_error_pos;
    struct vec i_error_att;
    struct vec i_error_q; // cable
    struct vec i_error_pl_att; // payload attitude

   // Cable PD 
    struct vec K_q;
    struct vec K_w;
    struct vec K_q_I;

   // Cable errors
    struct vec plp_error;
    struct vec plv_error;

   //Attitude PID
    struct vec KR;
    struct vec Komega;
    struct vec KI;

    // Logging variables
    struct vec qi_prev;
    struct vec qdi_prev;
    uint32_t qdi_prev_tick;
    struct vec payload_vel_prev;
    struct vec rpy;
    struct vec rpy_des;
    struct mat33 R_des;
    struct quat q;
    struct mat33 R;
    struct vec omega;
    struct vec omega_r;
    struct vec u;
    struct vec u_i;
    struct vec qidot_prev;
    struct vec acc_prev;
    struct vec F_d;
    struct vec M_d; // control moments of the payload
    struct quat qp_des; // desired quaternion orientation for the payload
    struct vec wp_des;  // desired omega for the payload
    struct vec omega_pr;
    struct vec qi;
    struct vec qidot;
    struct vec qdi;
    uint8_t en_qdidot; // 0: use qdidot = vzero(), 1: estimate numerically
    uint8_t en_accrb;
    struct vec qdidot;
    // desired value from the QP
    struct vec desVirtInp;
    struct vec desVirtInp2;
    struct vec desVirt2_prev;
    struct vec desVirt3_prev;
    uint32_t desVirtInp_tick;

    float lambdaa; // regularization on how close to stay to previous value
    float lambda_svm; // regularization value on how close the hyperplane should be to Fd

    uint8_t gen_hp; // 0: "old" geometric method, 1: "new" SVM method

    struct vec n1;
    struct vec n2;
    struct vec n3;
    struct vec n4;
    struct vec n5;
    struct vec n6;
    float radius;

    // osqp warm start aid
    struct osqp_warmstart_hyperplane_rb {
        float x[6];
        float y[8];
    } osqp_warmstart_hyperplane_rbs[3];

    struct osqp_warmstart_compute_Fd_pair {
        float x[9];
        float y[6];
    } osqp_warmstart_compute_Fd_pairs[3];

    struct osqp_warmstart_hyperplane {
        float x[4];
        float y[3];
    } osqp_warmstart_hyperplanes[3];

} controllerLeePayload_t;


void controllerLeePayloadInit(controllerLeePayload_t* self);
void controllerLeePayloadReset(controllerLeePayload_t* self);
void controllerLeePayload(controllerLeePayload_t* self, control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);

#ifdef CRAZYFLIE_FW
void controllerLeePayloadFirmwareInit(void);
bool controllerLeePayloadFirmwareTest(void);
void controllerLeePayloadFirmware(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);
#endif //   CRAZYFLIE_FW defined

#endif //__CONTROLLER_LEE_PAYLOAD_H__
