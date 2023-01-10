/*
The MIT License (MIT)

Copyright (c) 2022 Khaled Wahba 

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

TODO
*/

#include <math.h>
#include <string.h>
#include "math3d.h"
#include "controller_lee_payload.h"
#include "stdio.h"
#include "debug.h"
// QP
// #include "workspace_2uav_2hp.h"
#include "osqp.h"
extern OSQPWorkspace workspace_2uav_2hp;
extern OSQPWorkspace workspace_3uav_2hp;
extern OSQPWorkspace workspace_3uav_2hp_rig;
extern OSQPWorkspace workspace_2uav_1hp_rod;
extern OSQPWorkspace workspace_hyperplane;

#define GRAVITY_MAGNITUDE (9.81f)


struct QPInput
{
  struct vec F_d;
  struct vec M_d;
  struct vec plStPos;
  struct quat plStquat;
  struct vec statePos;
  struct vec statePos2;
  struct vec statePos3;
  uint8_t num_neighbors;
  uint8_t ids[2]; // ids for statePos2, statePos3
  controllerLeePayload_t* self;
  uint32_t timestamp; // ticks, i.e., ms
};

struct QPOutput
{
  struct vec desVirtInp;
  uint32_t timestamp; // ticks (copied from input)
};


#ifdef CRAZYFLIE_FW

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "static_mem.h"


#define CONTROLLER_LEE_PAYLOAD_QP_TASK_STACKSIZE (6 * configMINIMAL_STACK_SIZE)
#define CONTROLLER_LEE_PAYLOAD_QP_TASK_NAME "LEEQP"
#define CONTROLLER_LEE_PAYLOAD_QP_TASK_PRI 0

STATIC_MEM_TASK_ALLOC(controllerLeePayloadQPTask, CONTROLLER_LEE_PAYLOAD_QP_TASK_STACKSIZE);
static void controllerLeePayloadQPTask(void * prm);
static bool taskInitialized = false;

static QueueHandle_t queueQPInput;
STATIC_MEM_QUEUE_ALLOC(queueQPInput, 1, sizeof(struct QPInput));
static QueueHandle_t queueQPOutput;
STATIC_MEM_QUEUE_ALLOC(queueQPOutput, 1, sizeof(struct QPOutput));

static uint32_t qp_runtime_us = 0;

#endif

static inline struct vec computePlaneNormal(struct vec ps1, struct vec ps2, struct vec pload, float r, float l1, float l2) {
// Compute the normal of a plane, given the minimum desired distance r
// NEEDs TESTING!!!
  struct vec p1 = ps1;
  struct vec p2 = ps2;
  if (l1 <= l2) {
    p1 = ps1;
    p2 = ps2;
  }
  else {
    p1 = ps2;
    p2 = ps1;
  }
  struct vec p1_r = vsub(p1, pload);
  struct vec p2_r = vsub(p2, pload);

  float p1_dot_p2 = vdot(p1_r, p2_r);
  float normp1    = vmag(p1_r);
  float normp2    = vmag(p2_r);
  
  float angle_12  = acosf(p1_dot_p2/(normp1*normp2));
  struct vec p1_r_proj = mkvec(p1_r.x, p1_r.y, 0);
  float normp1_r_proj  = vmag(p1_r_proj);
  float angle_1 = M_PI_2_F;
  if (normp1_r_proj > 0) {
     angle_1 = acosf((vdot(p1_r, p1_r_proj))/(normp1*normp1_r_proj));
  }

  float angle_2 = M_PI_F - (angle_12 + angle_1);
  float normp2_r_new = normp1 * (sinf(angle_1)/sinf(angle_2));

  struct vec p2_new = vadd(pload, vscl(normp2_r_new, vnormalize(p2_r)));
  struct vec pos1 = ps1;
  struct vec pos2 = ps2;
  if(l2 >= l1) {
    pos2 = p2_new;
  }
  else{
    pos1 = p2_new;
  }

  struct vec mid = vscl(0.5, vsub(pos2, pos1));
  struct vec rvec = vscl(r, vnormalize(vsub(pos1, pos2)));
  struct vec pr = vadd(pos1, vadd(mid, rvec));

  struct vec p0pr = vsub(pr, pload);
  struct vec prp2 = vsub(pos2, pr);
  struct vec ns = vcross(prp2, p0pr);
  struct vec n_sol = vcross(p0pr, ns);
  return n_sol;
}


static controllerLeePayload_t g_self = {
  .mass = 0.034,
  .mp   = 0.01,
  // Inertia matrix (diagonal matrix), see
  // System Identification of the Crazyflie 2.0 Nano Quadrocopter
  // BA theses, Julian Foerster, ETHZ
  // https://polybox.ethz.ch/index.php/s/20dde63ee00ffe7085964393a55a91c7
  .J = {16.571710e-6, 16.655602e-6, 29.261652e-6}, // kg m^2

  // Position PID
  .Kpos_P = {18, 18, 18},
  .Kpos_P_limit = 100,
  .Kpos_D = {15, 15, 15},
  .Kpos_D_limit = 100,
  .Kpos_I ={10, 10, 10},
  .Kpos_I_limit = 0,

  // Payload attitude gains

  .Kprot_P = {0.01, 0.01, 0.01},
  .Kprot_P_limit = 100,
  .Kprot_D = {0.005, 0.005, 0.005},
  .Kprot_D_limit = 100,

  // Cables PD
  .K_q = {25, 25, 25},
  .K_w = {24, 24, 24},
  .K_q_I = {0, 0, 0},

  //Attitude PID 
  .KR = {0.008, 0.008, 0.01},
  .Komega = {0.0013, 0.0013, 0.002},
  .KI = {0.02, 0.02, 0.05},
  // desired orientation and omega for payload
  .qp_des = {0, 0, 0, 1},
  .wp_des = {0, 0, 0},

  .en_qdidot = 0,
  .en_accrb = 1,

  .radius = 0.15,

  .lambdaa = 0.0,
};

// static inline struct vec vclampscl(struct vec value, float min, float max) {
//   return mkvec(
//     clamp(value.x, min, max),
//     clamp(value.y, min, max),
//     clamp(value.z, min, max));
// }

static void runQP(const struct QPInput *input, struct QPOutput* output)
{
  struct vec F_d = input->F_d;
  struct vec M_d = input->M_d;
  uint8_t num_neighbors = input->num_neighbors;
  struct vec statePos = input->statePos;
  struct vec plStPos = input->plStPos;
  // struct quat plStquat = input->plStquat;
  float l1 = vmag(vsub(plStPos, statePos));
  float radius = input->self->radius;
  struct vec desVirtInp   = input->self->desVirtInp;
  struct vec desVirt_prev = input->self->desVirtInp;

  output->timestamp = input->timestamp;

  bool is_rigid_body = !isnanf(input->plStquat.w);
  
  if (num_neighbors >= 1) {
   // declare additional variables
    struct vec statePos2 = input->statePos2;
    float l2 = vmag(vsub(plStPos, statePos2));
    struct vec attPoint = input->self->attachement_points[0].point;
    struct vec attPoint2 = input->self->attachement_points[1].point;
    struct vec desVirt2_prev = input->self->desVirt2_prev;

    if (num_neighbors == 1) {
      if (is_rigid_body) {
        // QP for 2 uavs 1 hp, rigid rod will be added it here!
        // Set corresponding attachment points
        for (uint8_t i = 0; i < num_neighbors+1; ++i) {
          if (input->self->attachement_points[i].id == input->ids[0]) {
            attPoint2 = input->self->attachement_points[i].point;
          } else {
            attPoint = input->self->attachement_points[i].point;
          }
        }
        struct vec plSt_att = vadd(plStPos, qvrot(input->plStquat, attPoint));
        struct vec plSt_att2 = vadd(plStPos, qvrot(input->plStquat, attPoint2));
        
        l1 = vmag(vsub(plSt_att, statePos));
        l2 = vmag(vsub(plSt_att2, statePos2));
        M_d = mkvec(M_d.x, 0, M_d.z);
        struct vec n1 = computePlaneNormal(statePos, statePos2, plSt_att, radius, l1, l2);
        struct vec n2 = computePlaneNormal(statePos2, statePos, plSt_att2, radius, l2, l1);
        
        struct mat33 R0t = mtranspose(quat2rotmat(input->plStquat));

        struct mat33 attPR0t = mmul(mcrossmat(attPoint), R0t);
        struct mat33 attP2R0t = mmul(mcrossmat(attPoint2), R0t);

        OSQPWorkspace* workspace = &workspace_2uav_1hp_rod;
        workspace->settings->warm_start = 1;
        
        c_float Ax_new[48] = {
        R0t.m[0][0], R0t.m[1][0], R0t.m[2][0],  attPR0t.m[0][0],  attPR0t.m[1][0],  attPR0t.m[2][0], n1.x, 0,
        R0t.m[0][1], R0t.m[1][1], R0t.m[2][1],  attPR0t.m[0][1],  attPR0t.m[1][1],  attPR0t.m[2][1], n1.y, 0,
        R0t.m[0][2], R0t.m[1][2], R0t.m[2][2],  attPR0t.m[0][2],  attPR0t.m[1][2],  attPR0t.m[2][2], n1.z, 0,
        R0t.m[0][0], R0t.m[1][0], R0t.m[2][0], attP2R0t.m[0][0], attP2R0t.m[1][0], attP2R0t.m[2][0], 0, n2.x,
        R0t.m[0][1], R0t.m[1][1], R0t.m[2][1], attP2R0t.m[0][1], attP2R0t.m[1][1], attP2R0t.m[2][1], 0, n2.y,
        R0t.m[0][2], R0t.m[1][2], R0t.m[2][2], attP2R0t.m[0][2], attP2R0t.m[1][2], attP2R0t.m[2][2], 0, n2.z,
        };
        c_float Ax_new_n = 48;

        c_float l_new[9] =  {F_d.x,	F_d.y,	F_d.z, M_d.x, M_d.y, M_d.z, -INFINITY, -INFINITY,};
        c_float u_new[9] =  {F_d.x,	F_d.y,	F_d.z, M_d.x, M_d.y, M_d.z, 0, 0,};

        const float factor = - 2.0f * input->self->lambdaa / (1.0f + input->self->lambdaa);
        c_float q_new[6] = {factor * desVirt_prev.x,  factor * desVirt_prev.y,  factor * desVirt_prev.z,
                            factor * desVirt2_prev.x, factor * desVirt2_prev.y, factor * desVirt2_prev.z,
                          };

        osqp_update_A(workspace, Ax_new, OSQP_NULL, Ax_new_n);    
        osqp_update_lin_cost(workspace, q_new);
        osqp_update_lower_bound(workspace, l_new);
        osqp_update_upper_bound(workspace, u_new);
        osqp_solve(workspace);
        if (workspace->info->status_val == OSQP_SOLVED) {
          desVirtInp.x = (workspace)->solution->x[0];
          desVirtInp.y = (workspace)->solution->x[1];
          desVirtInp.z = (workspace)->solution->x[2];
          input->self->desVirt2_prev.x =  (workspace)->solution->x[3];
          input->self->desVirt2_prev.y =  (workspace)->solution->x[4];
          input->self->desVirt2_prev.z =  (workspace)->solution->x[5];
        } else {
        #ifdef CRAZYFLIE_FW
              DEBUG_PRINT("QP: %s\n", workspace->info->status);
        #else
              printf("QP: %s\n", workspace->info->status);
        #endif
            }
        input->self->n1 = n1;
        input->self->n2 = n2;
        output->desVirtInp = desVirtInp; 

      } else /* point mass case */ {
        // Solve QP for 2 uavs 1 hp point mass
        OSQPWorkspace* workspace = &workspace_2uav_2hp;
        workspace->settings->warm_start = 1;
        struct vec n1 = computePlaneNormal(statePos, statePos2, plStPos, radius, l1, l2);
        struct vec n2 = computePlaneNormal(statePos2, statePos, plStPos, radius, l2, l1);
       
        c_float Ax_new[12] = {1, n1.x, 1, n1.y, 1, n1.z, 1,  n2.x, 1, n2.y, 1, n2.z};  
        c_int Ax_new_n = 12;
        c_float l_new[6] =  {F_d.x,	F_d.y,	F_d.z, -INFINITY, -INFINITY,};
        c_float u_new[6] =  {F_d.x,	F_d.y,	F_d.z, 0, 0,};
        const float factor = - 2.0f * input->self->lambdaa / (1.0f + input->self->lambdaa);
        c_float q_new[6] = {factor * desVirt_prev.x,  factor * desVirt_prev.y,  factor * desVirt_prev.z,
                            factor * desVirt2_prev.x, factor * desVirt2_prev.y, factor * desVirt2_prev.z,
                          };
        
        osqp_update_A(workspace, Ax_new, OSQP_NULL, Ax_new_n);    
        osqp_update_lin_cost(workspace, q_new);
        osqp_update_lower_bound(workspace, l_new);
        osqp_update_upper_bound(workspace, u_new);
        osqp_solve(workspace);
        if (workspace->info->status_val == OSQP_SOLVED) {
          desVirtInp.x = (workspace)->solution->x[0];
          desVirtInp.y = (workspace)->solution->x[1];
          desVirtInp.z = (workspace)->solution->x[2];
          input->self->desVirt2_prev.x =  (workspace)->solution->x[3];
          input->self->desVirt2_prev.y =  (workspace)->solution->x[4];
          input->self->desVirt2_prev.z =  (workspace)->solution->x[5];
        } else {
        #ifdef CRAZYFLIE_FW
              DEBUG_PRINT("QP: %s\n", workspace->info->status);
        #else
              printf("QP: %s\n", workspace->info->status);
        #endif
            }
        input->self->n1 = n1;
        input->self->n2 = n2;
        output->desVirtInp = desVirtInp; 
      }
    } else if (num_neighbors >= 2) {
     // declare additional variables
      struct vec statePos3 = input->statePos3;     
      float l3 = vmag(vsub(plStPos, statePos3));
      struct vec attPoint3 = input->self->attachement_points[2].point;  
      struct vec desVirt3_prev = input->self->desVirt3_prev;

      if (num_neighbors == 2) {
        if (is_rigid_body) {
          // solve QP for 3 uavs 2 hps and rigid triangle payload
          for (uint8_t i = 0; i < num_neighbors+1; ++i) {
            if (input->self->attachement_points[i].id == input->ids[0]) {
              attPoint2 = input->self->attachement_points[i].point;
            } else if (input->self->attachement_points[i].id == input->ids[1]) {
              attPoint3 = input->self->attachement_points[i].point;
            } else {
              attPoint = input->self->attachement_points[i].point;
            }
          }
          OSQPWorkspace* workspace = &workspace_3uav_2hp_rig;
          workspace->settings->warm_start = 1;

          c_float x_warm[9] = {  desVirt_prev.x,    desVirt_prev.y,    desVirt_prev.z,
                                desVirt2_prev.x,   desVirt2_prev.y,   desVirt2_prev.z, 
                                desVirt3_prev.x,   desVirt3_prev.y,   desVirt3_prev.z};
          osqp_warm_start_x(workspace, x_warm);
          struct vec plSt_att = vadd(plStPos, qvrot(input->plStquat, attPoint));
          struct vec plSt_att2 = vadd(plStPos, qvrot(input->plStquat, attPoint2));
          struct vec plSt_att3 = vadd(plStPos, qvrot(input->plStquat, attPoint3));
          l1 = vmag(vsub(plSt_att, statePos));
          l2 = vmag(vsub(plSt_att2, statePos2));
          l3 = vmag(vsub(plSt_att3, statePos3));

          struct vec n1 = computePlaneNormal(statePos, statePos2, plSt_att,  radius, l1, l2);
          struct vec n2 = computePlaneNormal(statePos, statePos3, plSt_att,  radius, l1, l3);
          struct vec n3 = computePlaneNormal(statePos2, statePos, plSt_att2,  radius, l2, l1);
          struct vec n4 = computePlaneNormal(statePos2, statePos3, plSt_att2, radius, l2, l3);
          struct vec n5 = computePlaneNormal(statePos3, statePos, plSt_att3,  radius, l3, l1);
          struct vec n6 = computePlaneNormal(statePos3, statePos2, plSt_att3, radius, l3, l2);

          c_float Ax_new[45] = {
            1, attPoint.z, -attPoint.y, n1.x, n2.x, 1, -attPoint.z, attPoint.x, n1.y, n2.y, 1, attPoint.y, -attPoint.x, n1.z, n2.z,
            1, attPoint2.z, -attPoint2.y, n3.x, n4.x, 1, -attPoint2.z, attPoint2.x, n3.y, n4.y, 1, attPoint2.y, -attPoint2.x, n3.z, n4.z,
            1, attPoint3.z, -attPoint3.y, n5.x, n6.x, 1, -attPoint3.z, attPoint3.x, n5.y, n6.y, 1, attPoint3.y, -attPoint3.x, n5.z, n6.z,
          };
          c_int Ax_new_n = 45;
          c_float l_new[12] =  {F_d.x,	F_d.y,	F_d.z,  M_d.x,  M_d.y,  M_d.z,  -INFINITY, -INFINITY, -INFINITY, -INFINITY, -INFINITY, -INFINITY,};
          c_float u_new[12] =  {F_d.x,	F_d.y,	F_d.z,  M_d.x,  M_d.y,  M_d.z, 0, 0,  0, 0,  0, 0};

          /* P = np.eye(9) [can't be changed online]
             x^2 + lambda (x^2 - 2xx_d + x_d^2)
             => J = (1+lambda) x^2 - 2 * lambda x_d x
             =>   = x^2 - 2 * lambda / (1+lambda) x_d x
             => q = -2 * lambda / (1+lambda) * x_d
          */
          const float factor = - 2.0f * input->self->lambdaa / (1.0f + input->self->lambdaa);

          c_float q_new[9] = {factor * desVirt_prev.x,  factor * desVirt_prev.y,  factor * desVirt_prev.z,
                              factor * desVirt2_prev.x, factor * desVirt2_prev.y, factor * desVirt2_prev.z, 
                              factor * desVirt3_prev.x, factor * desVirt3_prev.y, factor * desVirt3_prev.z};

          osqp_update_A(workspace, Ax_new, OSQP_NULL, Ax_new_n);    
          osqp_update_lin_cost(workspace, q_new);
          osqp_update_lower_bound(workspace, l_new);
          osqp_update_upper_bound(workspace, u_new);
          osqp_solve(workspace);

          if (workspace->info->status_val == OSQP_SOLVED) {
            desVirtInp.x = (workspace)->solution->x[0];
            desVirtInp.y = (workspace)->solution->x[1];
            desVirtInp.z = (workspace)->solution->x[2];
            
            input->self->desVirt2_prev.x =  (workspace)->solution->x[3];
            input->self->desVirt2_prev.y =  (workspace)->solution->x[4];
            input->self->desVirt2_prev.z =  (workspace)->solution->x[5];
            input->self->desVirt3_prev.x =  (workspace)->solution->x[6];
            input->self->desVirt3_prev.y =  (workspace)->solution->x[7];
            input->self->desVirt3_prev.z =  (workspace)->solution->x[8];
          } else {
          #ifdef CRAZYFLIE_FW
                DEBUG_PRINT("QP: %s\n", workspace->info->status);
          #else
                printf("QP: %s\n", workspace->info->status);
          #endif
              }
          input->self->n1 = n1;
          input->self->n2 = n2;
          input->self->n3 = n3;
          input->self->n4 = n4;
          input->self->n5 = n5;
          input->self->n6 = n6;
          output->desVirtInp = desVirtInp;
        } else /* point mass case */ {
          // solve QP for 3 uavs 2 hps and point mass
          OSQPWorkspace* workspace = &workspace_3uav_2hp;
          workspace->settings->warm_start = 1;
          struct vec n1 = computePlaneNormal(statePos, statePos2, plStPos,  radius, l1, l2);
          struct vec n2 = computePlaneNormal(statePos, statePos3, plStPos,  radius, l1, l3);
          struct vec n3 = computePlaneNormal(statePos2, statePos, plStPos,  radius, l2, l1);
          struct vec n4 = computePlaneNormal(statePos2, statePos3, plStPos, radius, l2, l3);
          struct vec n5 = computePlaneNormal(statePos3, statePos, plStPos,  radius, l3, l1);
          struct vec n6 = computePlaneNormal(statePos3, statePos2, plStPos, radius, l3, l2);

          c_float Ax_new[27] = {1, n1.x, n2.x, 1, n1.y, n2.y, 1, n1.z, n2.z, 1, n3.x, n4.x, 1, n3.y, n4.y, 1, n3.z, n4.z, 1, n5.x, n6.x, 1, n5.y, n6.y, 1, n5.z, n6.z, };
          c_int Ax_new_n = 27;
          c_float l_new[9] =  {F_d.x,	F_d.y,	F_d.z, -INFINITY, -INFINITY, -INFINITY, -INFINITY, -INFINITY, -INFINITY,};
          c_float u_new[9] =  {F_d.x,	F_d.y,	F_d.z, 0, 0,  0, 0,  0, 0};
          const float factor = - 2.0f * input->self->lambdaa / (1.0f + input->self->lambdaa);

          c_float q_new[9] = {factor * desVirt_prev.x,  factor * desVirt_prev.y,  factor * desVirt_prev.z,
                              factor * desVirt2_prev.x, factor * desVirt2_prev.y, factor * desVirt2_prev.z, 
                              factor * desVirt3_prev.x, factor * desVirt3_prev.y, factor * desVirt3_prev.z};

          osqp_update_A(workspace, Ax_new, OSQP_NULL, Ax_new_n);    
          osqp_update_lin_cost(workspace, q_new);
          osqp_update_lower_bound(workspace, l_new);
          osqp_update_upper_bound(workspace, u_new);
          osqp_solve(workspace);

          if (workspace->info->status_val == OSQP_SOLVED) {
            desVirtInp.x = (workspace)->solution->x[0];
            desVirtInp.y = (workspace)->solution->x[1];
            desVirtInp.z = (workspace)->solution->x[2];
            
            input->self->desVirt2_prev.x =  (workspace)->solution->x[3];
            input->self->desVirt2_prev.y =  (workspace)->solution->x[4];
            input->self->desVirt2_prev.z =  (workspace)->solution->x[5];
            input->self->desVirt3_prev.x =   (workspace)->solution->x[6];
            input->self->desVirt3_prev.y =   (workspace)->solution->x[7];
            input->self->desVirt3_prev.z =   (workspace)->solution->x[8];
          } else {
          #ifdef CRAZYFLIE_FW
                DEBUG_PRINT("QP: %s\n", workspace->info->status);
          #else
                printf("QP: %s\n", workspace->info->status);
          #endif
              }
          input->self->n1 = n1;
          input->self->n2 = n2;
          input->self->n3 = n3;
          input->self->n4 = n4;
          input->self->n5 = n5;
          input->self->n6 = n6;
          output->desVirtInp = desVirtInp;
        }
      }
    }
  }
}
#ifdef CRAZYFLIE_FW

static struct vec computeDesiredVirtualInput(controllerLeePayload_t* self, const state_t *state, struct vec F_d, struct vec M_d, uint32_t* ticks)
{
  struct QPInput qpinput;
  struct QPOutput qpoutput;

  // push the latest change to the QP
  qpinput.F_d = F_d;
  qpinput.M_d = M_d;
  qpinput.plStPos = mkvec(state->payload_pos.x, state->payload_pos.y, state->payload_pos.z);
  qpinput.plStquat = mkquat(state->payload_quat.x, state->payload_quat.y, state->payload_quat.z, state->payload_quat.w);
  if (state->num_neighbors == 1) {
    struct vec rpy = quat2rpy(qpinput.plStquat);
    rpy.y = 0;
    qpinput.plStquat = rpy2quat(rpy);
  }

  qpinput.statePos = mkvec(state->position.x, state->position.y, state->position.z);
    // We assume that we always have at least 1 neighbor
  qpinput.statePos2 = mkvec(state->neighbors[0].pos.x, state->neighbors[0].pos.y, state->neighbors[0].pos.z);
  qpinput.ids[0] = state->neighbors[0].id;
  qpinput.num_neighbors = state->num_neighbors;
  if (state->num_neighbors == 2) 
  {
    qpinput.statePos3 = mkvec(state->neighbors[1].pos.x, state->neighbors[1].pos.y, state->neighbors[1].pos.z);
    qpinput.ids[1] = state->neighbors[1].id;
  }  

  qpinput.self = self;
  qpinput.timestamp = *ticks;
  xQueueOverwrite(queueQPInput, &qpinput);

  // get the latest result from the async computation (wait until at least one computation has been made)
  xQueuePeek(queueQPOutput, &qpoutput, portMAX_DELAY);

  *ticks = qpoutput.timestamp;
  return qpoutput.desVirtInp;
}

void controllerLeePayloadQPTask(void * prm)
{
  struct QPInput qpinput;
  struct QPOutput qpoutput;

  while(1) {
    // wait until we get the next request
    xQueueReceive(queueQPInput, &qpinput, portMAX_DELAY);

    // solve the QP
    uint64_t start = usecTimestamp();
    runQP(&qpinput, &qpoutput);
    uint64_t end = usecTimestamp();
    qp_runtime_us = end - start;
    // store the result
    xQueueOverwrite(queueQPOutput, &qpoutput);
  }
}
#else

static struct vec computeDesiredVirtualInput(controllerLeePayload_t* self, const state_t *state, struct vec F_d, struct vec M_d, uint32_t* ticks)
{
  struct QPInput qpinput;
  struct QPOutput qpoutput;

  // push the latest change to the QP
  qpinput.F_d = F_d;
  qpinput.M_d = M_d; 
  qpinput.plStPos = mkvec(state->payload_pos.x, state->payload_pos.y, state->payload_pos.z);
  qpinput.plStquat = mkquat(state->payload_quat.x, state->payload_quat.y, state->payload_quat.z, state->payload_quat.w);
  if (state->num_neighbors == 1) {
    struct vec rpy = quat2rpy(qpinput.plStquat);
    rpy.y = 0;
    qpinput.plStquat = rpy2quat(rpy);
  }
  qpinput.statePos = mkvec(state->position.x, state->position.y, state->position.z);

  // We assume that we always have at least 1 neighbor
  qpinput.statePos2 = mkvec(state->neighbors[0].pos.x, state->neighbors[0].pos.y, state->neighbors[0].pos.z);
  qpinput.ids[0] = state->neighbors[0].id;
  qpinput.num_neighbors = state->num_neighbors;
  if (state->num_neighbors == 2) 
  {
    qpinput.statePos3 = mkvec(state->neighbors[1].pos.x, state->neighbors[1].pos.y, state->neighbors[1].pos.z);
    qpinput.ids[1] = state->neighbors[1].id;
  }  
  qpinput.self = self;
  qpinput.timestamp = *ticks;
  // solve the QP
  runQP(&qpinput, &qpoutput);

  *ticks = qpoutput.timestamp;
  return qpoutput.desVirtInp;
}

#endif

void controllerLeePayloadReset(controllerLeePayload_t* self)
{
  self->i_error_pos = vzero();
  self->i_error_att = vzero();
  self->i_error_q = vzero();
  self->qi_prev = mkvec(0,0,-1);
  self->qidot_prev = vzero();
  self->acc_prev   = vzero();
  self->payload_vel_prev = vzero();
  self->qdi_prev = vzero();
}

void controllerLeePayloadInit(controllerLeePayload_t* self)
{
  // copy default values (bindings), or NOP (firmware)
  *self = g_self;

#ifdef CRAZYFLIE_FW
  if (!taskInitialized) {
    STATIC_MEM_TASK_CREATE(controllerLeePayloadQPTask, controllerLeePayloadQPTask, CONTROLLER_LEE_PAYLOAD_QP_TASK_NAME, NULL, CONTROLLER_LEE_PAYLOAD_QP_TASK_PRI);

    queueQPInput = STATIC_MEM_QUEUE_CREATE(queueQPInput);
    queueQPOutput = STATIC_MEM_QUEUE_CREATE(queueQPOutput);

    taskInitialized = true;
  }
#endif

  controllerLeePayloadReset(self);
}

bool controllerLeePayloadTest(controllerLeePayload_t* self)
{
  return true;
}

void controllerLeePayload(controllerLeePayload_t* self, control_t *control, setpoint_t *setpoint,
                                                        const sensorData_t *sensors,
                                                        const state_t *state,
                                                        const uint32_t tick)
{
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }

  // uint64_t startTime = usecTimestamp();

  float dt = (float)(1.0f/ATTITUDE_RATE);
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

  // Position controller
  if (   setpoint->mode.x == modeAbs
      || setpoint->mode.y == modeAbs
      || setpoint->mode.z == modeAbs) {
    
    struct vec plPos_d = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
    struct vec plVel_d = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
    struct vec plAcc_d = mkvec(setpoint->acceleration.x, setpoint->acceleration.y, setpoint->acceleration.z + GRAVITY_MAGNITUDE);

    struct vec statePos = mkvec(state->position.x, state->position.y, state->position.z);
    struct vec stateVel = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);
    struct vec plStPos = mkvec(state->payload_pos.x, state->payload_pos.y, state->payload_pos.z);
    struct vec plStVel = mkvec(state->payload_vel.x, state->payload_vel.y, state->payload_vel.z);

    // rotational states of the payload
    struct quat plquat = mkquat(state->payload_quat.x, state->payload_quat.y, state->payload_quat.z, state->payload_quat.w);
    struct vec plomega = mkvec(state->payload_omega.x, state->payload_omega.y, state->payload_omega.z);
    if (state->num_neighbors == 1) {
      struct vec rpy = quat2rpy(plquat);
      rpy.y = 0;
      plquat = rpy2quat(rpy);
      plomega.y = 0;
    }

    // errors
    struct vec plpos_e = vclampnorm(vsub(plPos_d, plStPos), self->Kpos_P_limit);
    struct vec plvel_e = vclampnorm(vsub(plVel_d, plStVel), self->Kpos_D_limit);
    self->i_error_pos = vclampnorm(vadd(self->i_error_pos, vscl(dt, plpos_e)), self->Kpos_I_limit);

    struct vec attPoint = mkvec(0, 0, 0);
    if (!isnanf(plquat.w)) {
      
      // find the attachment point for this UAV (the one, which doesn't have any neighbor associated with it)
      for (uint8_t i = 0; i < state->num_neighbors+1; ++i) {
        bool found = false;
        for (uint8_t j = 0; j < state->num_neighbors; ++j) {
          if (self->attachement_points[i].id == state->neighbors[j].id) {
            // this attachement point belongs to a neighbor
            found = true;
            break;
          }
        }
        if (!found) {
          attPoint = self->attachement_points[i].point;
          break;
        }
      }
      // If the payload is a rigid body then the the attachment point should be added to PlStPos
      plStPos = vadd(plStPos, qvrot(plquat, attPoint));
    }
    float l = vmag(vsub(plStPos, statePos));


    // payload orientation errors
    // payload quat to R 
    struct mat33 Rp = quat2rotmat(plquat);
    // define desired payload Rp_des = eye(3) (i.e., qp_des = [0,0,0,1] (x,y,z,w))
    self->qp_des = mkquat(setpoint->attitudeQuaternion.x, setpoint->attitudeQuaternion.y, setpoint->attitudeQuaternion.z, setpoint->attitudeQuaternion.w);
    struct mat33 Rp_des = quat2rotmat(self->qp_des); 
    // define orientation error     
    // eRp =  msub(mmul(mtranspose(self->R_des), self->R), mmul(mtranspose(self->R), self->R_des));
    struct mat33 eRMp =  msub(mmul(mtranspose(Rp_des), Rp), mmul(mtranspose(Rp), Rp_des));
    struct vec eRp = vscl(0.5f, mkvec(eRMp.m[2][1], eRMp.m[0][2], eRMp.m[1][0]));

    self->wp_des = mkvec(radians(setpoint->attitudeRate.roll), radians(setpoint->attitudeRate.pitch), radians(setpoint->attitudeRate.yaw));
    self->omega_pr = mvmul(mmul(mtranspose(Rp), Rp_des), self->wp_des);
    struct vec omega_perror = vsub(plomega, self->omega_pr);

    self->plp_error = plpos_e;
    self->plv_error = plvel_e;

    self->F_d =vscl(self->mp ,vadd4(
      plAcc_d,
      veltmul(self->Kpos_P, plpos_e),
      veltmul(self->Kpos_D, plvel_e),
      veltmul(self->Kpos_I, self->i_error_pos)));

    self->M_d = vadd(
      vneg(veltmul(self->Kprot_P, eRp)),
      vneg(veltmul(self->Kprot_D, omega_perror))
    );
    if (!isnanf(plquat.w)) {
      // TODO: move that into the QP
      struct vec F_dP = mvmul(mtranspose(Rp),self->F_d);
      self->desVirtInp_tick = tick;
      self->desVirtInp = computeDesiredVirtualInput(self, state, F_dP, self->M_d, &self->desVirtInp_tick);
    }
    else{
      self->desVirtInp_tick = tick;
      self->desVirtInp = computeDesiredVirtualInput(self, state, self->F_d, self->M_d, &self->desVirtInp_tick);
    }
    // computed desired generalized forces in rigid payload case for equation 23 is Pmu_des = [Rp.T@F_d, M_d]
    // if a point mass for the payload is considered then: Pmu_des = F_d

    //directional unit vector qi and angular velocity wi pointing from UAV to payload
    self->qi = vnormalize(vsub(plStPos, statePos)); 

    // from the text between (2) and (3) in Lee's paper
    // qi_dot = (x0_dot + R0_dot rho_i - xi_dot)/li
    if (!isnanf(plquat.w)) {
      struct mat33 R0_dot = mmul(Rp, mcrossmat(plomega));
      self->qidot = vdiv(vsub(vadd(plStVel, mvmul(R0_dot, attPoint)), stateVel), l);
    } else {
      self->qidot = vdiv(vsub(plStVel, stateVel), l);
    }

    struct vec wi = vcross(self->qi, self->qidot);
    struct mat33 qiqiT = vecmult(self->qi);
    struct vec virtualInp = mvmul(qiqiT,self->desVirtInp);

    
    // Compute parallel component
    struct vec acc_ = plAcc_d;
    if (!isnanf(plquat.w)) {
      if (self->en_accrb) {
        acc_ = vadd(plAcc_d, qvrot(plquat, mvmul(mmul(mcrossmat(plomega), mcrossmat(plomega)), attPoint)));
      }
    } 
    struct vec u_parallel = vadd3(virtualInp, vscl(self->mass*l*vmag2(wi), self->qi), vscl(self->mass, mvmul(qiqiT, acc_)));
    
    // Compute Perpindicular Component
    struct vec qdi = vneg(vnormalize(self->desVirtInp));
    struct vec eq  = vcross(qdi, self->qi);
    self->i_error_q = vadd(self->i_error_q, vscl(dt, eq));
    struct mat33 skewqi = mcrossmat(self->qi);
    struct mat33 skewqi2 = mmul(skewqi,skewqi);

    self->qdidot = vzero();
    if (self->en_qdidot) {
      float qdi_dt = (self->desVirtInp_tick - self->qdi_prev_tick) / 1000.0f;
      if (qdi_dt > 0) {
        self->qdidot = vdiv(vsub(qdi, self->qdi_prev), qdi_dt);
      }
    }

    if (self->desVirtInp_tick != self->qdi_prev_tick) {
      self->qdi_prev = qdi;
      self->qdi_prev_tick = self->desVirtInp_tick;
    }
    struct vec wdi = vcross(qdi, self->qdidot);
    struct vec ew = vadd(wi, mvmul(skewqi2, wdi));

    struct vec u_perpind = vsub(
      vscl(self->mass*l, mvmul(skewqi, vadd4(
        vneg(veltmul(self->K_q, eq)),
        vneg(veltmul(self->K_w, ew)),
        vneg(veltmul(self->K_q_I, self->i_error_q)), 
        vneg(vscl(vdot(self->qi, wdi), self->qidot))))),
      vscl(self->mass, mvmul(skewqi2, acc_))
    );

    self->u_i = vadd(u_parallel, u_perpind);
    
    control->thrustSI = vmag(self->u_i);
    control->u_all[0] = self->u_i.x;
    control->u_all[1] = self->u_i.y;
    control->u_all[2] = self->u_i.z;


    self->thrustSI = control->thrustSI;
  //  Reset the accumulated error while on the ground
    if (control->thrustSI < 0.01f) {
      controllerLeePayloadReset(self);
    }
  
  self->q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
  self->rpy = quat2rpy(self->q);
  self->R = quat2rotmat(self->q);

  // Compute Desired Rotation matrix
    struct vec Fd_ = self->u_i;
    struct vec xdes = vbasis(0);
    struct vec ydes = vbasis(1);
    struct vec zdes = vbasis(2);
   
    if (self->thrustSI > 0) {
      zdes = vnormalize(Fd_);
    } 
    // struct vec xcdes = mkvec(cosf(desiredYaw), sinf(desiredYaw), 0); 
    struct vec xcdes = mkvec(1, 0, 0); 
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
          control->thrustSI  = 0;
          control->torque[0] = 0;
          control->torque[1] = 0;
          control->torque[2] = 0;
          controllerLeePayloadReset(self);
          return;
      }
    }
    // On CF2, thrust is mapped 65536 <==> 4 * 12 grams
    const float max_thrust = 70.0f / 1000.0f * 9.81f; // N
    control->thrustSI = setpoint->thrust / UINT16_MAX * max_thrust;

    self->qr = mkvec(
      radians(setpoint->attitude.roll),
      -radians(setpoint->attitude.pitch), // This is in the legacy coordinate system where pitch is inverted
      desiredYaw);
  }

  // Attitude controller

  // current rotation [R]
  self->q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
  self->rpy = quat2rpy(self->q);
  self->R = quat2rotmat(self->q);

  // desired rotation [Rdes]
  struct quat q_des = mat2quat(self->R_des);
  self->rpy_des = quat2rpy(q_des);

  // rotation error
  struct mat33 eRM = msub(mmul(mtranspose(self->R_des), self->R), mmul(mtranspose(self->R), self->R_des));

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

  if (control->thrustSI != 0) {
    struct vec tmp = vsub(desJerk, vscl(vdot(zdes, desJerk), zdes));
    hw = vscl(self->mass/control->thrustSI, tmp);
  }

  struct vec z_w = mkvec(0, 0, 1);
  float desiredYawRate = radians(setpoint->attitudeRate.yaw) * vdot(zdes, z_w);
  struct vec omega_des = mkvec(-vdot(hw,ydes), vdot(hw,xdes), desiredYawRate);
  
  self->omega_r = mvmul(mmul(mtranspose(self->R), self->R_des), omega_des);

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

  // if (enableNN > 1) {
  //   u = vsub(u, tau_a);
  // }

  control->controlMode = controlModeForceTorque;
  control->torque[0] = self->u.x;
  control->torque[1] = self->u.y;
  control->torque[2] = self->u.z;

  // ticks = usecTimestamp() - startTime;
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

void controllerLeePayloadFirmware(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  controllerLeePayload(&g_self, control, setpoint, sensors, state, tick);  
}

PARAM_GROUP_START(ctrlLeeP)
PARAM_ADD(PARAM_FLOAT, KR_x, &g_self.KR.x)
PARAM_ADD(PARAM_FLOAT, KR_y, &g_self.KR.y)
PARAM_ADD(PARAM_FLOAT, KR_z, &g_self.KR.z)
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

// Attitude Payload P
PARAM_ADD(PARAM_FLOAT, Kprot_Px, &g_self.Kprot_P.x)
PARAM_ADD(PARAM_FLOAT, Kprot_Py, &g_self.Kprot_P.y)
PARAM_ADD(PARAM_FLOAT, Kprot_Pz, &g_self.Kprot_P.z)
PARAM_ADD(PARAM_FLOAT, Kprot_P_limit, &g_self.Kprot_P_limit)
// Attitude Payload D
PARAM_ADD(PARAM_FLOAT, Kprot_Dx, &g_self.Kprot_D.x)
PARAM_ADD(PARAM_FLOAT, Kprot_Dy, &g_self.Kprot_D.y)
PARAM_ADD(PARAM_FLOAT, Kprot_Dz, &g_self.Kprot_D.z)
PARAM_ADD(PARAM_FLOAT, Kprot_D_limit, &g_self.Kprot_D_limit)

// Attitude P
PARAM_ADD(PARAM_FLOAT, KRx, &g_self.KR.x)
PARAM_ADD(PARAM_FLOAT, KRy, &g_self.KR.y)
PARAM_ADD(PARAM_FLOAT, KRz, &g_self.KR.z)

// Attitude D
PARAM_ADD(PARAM_FLOAT, Komx, &g_self.Komega.x)
PARAM_ADD(PARAM_FLOAT, Komy, &g_self.Komega.y)
PARAM_ADD(PARAM_FLOAT, Komz, &g_self.Komega.z)

// Attitude I
PARAM_ADD(PARAM_FLOAT, KI_x, &g_self.KI.x)
PARAM_ADD(PARAM_FLOAT, KI_y, &g_self.KI.y)
PARAM_ADD(PARAM_FLOAT, KI_z, &g_self.KI.z)

// Cable P
PARAM_ADD(PARAM_FLOAT, Kqx, &g_self.K_q.x)
PARAM_ADD(PARAM_FLOAT, Kqy, &g_self.K_q.y)
PARAM_ADD(PARAM_FLOAT, Kqz, &g_self.K_q.z)

// Cable D
PARAM_ADD(PARAM_FLOAT, Kwx, &g_self.K_w.x)
PARAM_ADD(PARAM_FLOAT, Kwy, &g_self.K_w.y)
PARAM_ADD(PARAM_FLOAT, Kwz, &g_self.K_w.z)

// Cable I
PARAM_ADD(PARAM_FLOAT, KqIx, &g_self.K_q_I.x)
PARAM_ADD(PARAM_FLOAT, KqIy, &g_self.K_q_I.y)
PARAM_ADD(PARAM_FLOAT, KqIz, &g_self.K_q_I.z)

PARAM_ADD(PARAM_UINT8, en_qdidot, &g_self.en_qdidot)
PARAM_ADD(PARAM_UINT8, en_accrb, &g_self.en_accrb)

PARAM_ADD(PARAM_FLOAT, mass, &g_self.mass)
PARAM_ADD(PARAM_FLOAT, massP, &g_self.mp)

PARAM_ADD(PARAM_FLOAT, radius, &g_self.radius)

// QP tuning

PARAM_ADD(PARAM_FLOAT, lambda, &g_self.lambdaa)

// Attachement points rigid body payload
PARAM_ADD(PARAM_UINT8, ap0id, &g_self.attachement_points[0].id)
PARAM_ADD(PARAM_FLOAT, ap0x, &g_self.attachement_points[0].point.x)
PARAM_ADD(PARAM_FLOAT, ap0y, &g_self.attachement_points[0].point.y)
PARAM_ADD(PARAM_FLOAT, ap0z, &g_self.attachement_points[0].point.z)

PARAM_ADD(PARAM_UINT8, ap1id, &g_self.attachement_points[1].id)
PARAM_ADD(PARAM_FLOAT, ap1x, &g_self.attachement_points[1].point.x)
PARAM_ADD(PARAM_FLOAT, ap1y, &g_self.attachement_points[1].point.y)
PARAM_ADD(PARAM_FLOAT, ap1z, &g_self.attachement_points[1].point.z)

PARAM_ADD(PARAM_UINT8, ap2id, &g_self.attachement_points[2].id)
PARAM_ADD(PARAM_FLOAT, ap2x, &g_self.attachement_points[2].point.x)
PARAM_ADD(PARAM_FLOAT, ap2y, &g_self.attachement_points[2].point.y)
PARAM_ADD(PARAM_FLOAT, ap2z, &g_self.attachement_points[2].point.z)


PARAM_GROUP_STOP(ctrlLeeP)


LOG_GROUP_START(ctrlLeeP)

LOG_ADD(LOG_FLOAT, thrustSI, &g_self.thrustSI)
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

// desired quat for the payload
LOG_ADD(LOG_FLOAT, qp_desx, &g_self.qp_des.x)
LOG_ADD(LOG_FLOAT, qp_desy, &g_self.qp_des.y)
LOG_ADD(LOG_FLOAT, qp_desz, &g_self.qp_des.z)
LOG_ADD(LOG_FLOAT, qp_desw, &g_self.qp_des.w)

// desired omega for payload
LOG_ADD(LOG_FLOAT, omega_prx, &g_self.omega_pr.x)
LOG_ADD(LOG_FLOAT, omega_pry, &g_self.omega_pr.y)
LOG_ADD(LOG_FLOAT, omega_prz, &g_self.omega_pr.z)

// omega
LOG_ADD(LOG_FLOAT, omegax, &g_self.omega.x)
LOG_ADD(LOG_FLOAT, omegay, &g_self.omega.y)
LOG_ADD(LOG_FLOAT, omegaz, &g_self.omega.z)

// omega_r
LOG_ADD(LOG_FLOAT, omegarx, &g_self.omega_r.x)
LOG_ADD(LOG_FLOAT, omegary, &g_self.omega_r.y)
LOG_ADD(LOG_FLOAT, omegarz, &g_self.omega_r.z)

LOG_ADD(LOG_FLOAT, ux, &g_self.u_i.x)
LOG_ADD(LOG_FLOAT, uy, &g_self.u_i.y)
LOG_ADD(LOG_FLOAT, uz, &g_self.u_i.z)
// Cable States
LOG_ADD(LOG_FLOAT, qix, &g_self.qi.x)
LOG_ADD(LOG_FLOAT, qiy, &g_self.qi.y)
LOG_ADD(LOG_FLOAT, qiz, &g_self.qi.z)

LOG_ADD(LOG_FLOAT, qidotx, &g_self.qidot.x)
LOG_ADD(LOG_FLOAT, qidoty, &g_self.qidot.y)
LOG_ADD(LOG_FLOAT, qidotz, &g_self.qidot.z)

LOG_ADD(LOG_FLOAT, qdidotx, &g_self.qdidot.x)
LOG_ADD(LOG_FLOAT, qdidoty, &g_self.qdidot.y)
LOG_ADD(LOG_FLOAT, qdidotz, &g_self.qdidot.z)

// hyperplanes
LOG_ADD(LOG_FLOAT, n1x, &g_self.n1.x)
LOG_ADD(LOG_FLOAT, n1y, &g_self.n1.y)
LOG_ADD(LOG_FLOAT, n1z, &g_self.n1.z)

LOG_ADD(LOG_FLOAT, n2x, &g_self.n2.x)
LOG_ADD(LOG_FLOAT, n2y, &g_self.n2.y)
LOG_ADD(LOG_FLOAT, n2z, &g_self.n2.z)

LOG_ADD(LOG_FLOAT, n3x, &g_self.n3.x)
LOG_ADD(LOG_FLOAT, n3y, &g_self.n3.y)
LOG_ADD(LOG_FLOAT, n3z, &g_self.n3.z)

LOG_ADD(LOG_FLOAT, n4x, &g_self.n4.x)
LOG_ADD(LOG_FLOAT, n4y, &g_self.n4.y)
LOG_ADD(LOG_FLOAT, n4z, &g_self.n4.z)

LOG_ADD(LOG_FLOAT, n5x, &g_self.n5.x)
LOG_ADD(LOG_FLOAT, n5y, &g_self.n5.y)
LOG_ADD(LOG_FLOAT, n5z, &g_self.n5.z)

LOG_ADD(LOG_FLOAT, n6x, &g_self.n6.x)
LOG_ADD(LOG_FLOAT, n6y, &g_self.n6.y)
LOG_ADD(LOG_FLOAT, n6z, &g_self.n6.z)

// computed desired payload force
LOG_ADD(LOG_FLOAT, Fdx, &g_self.F_d.x)
LOG_ADD(LOG_FLOAT, Fdy, &g_self.F_d.y)
LOG_ADD(LOG_FLOAT, Fdz, &g_self.F_d.z)

LOG_ADD(LOG_FLOAT, Mdx, &g_self.M_d.x)
LOG_ADD(LOG_FLOAT, Mdy, &g_self.M_d.y)
LOG_ADD(LOG_FLOAT, Mdz, &g_self.M_d.z)

// computed virtual input
LOG_ADD(LOG_FLOAT, desVirtInpx, &g_self.desVirtInp.x)
LOG_ADD(LOG_FLOAT, desVirtInpy, &g_self.desVirtInp.y)
LOG_ADD(LOG_FLOAT, desVirtInpz, &g_self.desVirtInp.z)

LOG_ADD(LOG_UINT32, profQP, &qp_runtime_us)

LOG_GROUP_STOP(ctrlLeeP)

#endif // CRAZYFLIE_FW defined


               


          
