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
#include "scaling.h"
#include "auxil.h"
extern OSQPWorkspace workspace_2uav_2hp;
extern OSQPWorkspace workspace_3uav_2hp;
extern OSQPWorkspace workspace_3uav_2hp_rig;
extern OSQPWorkspace workspace_2uav_1hp_rod;
extern OSQPWorkspace workspace_hyperplane;
extern OSQPWorkspace workspace_hyperplane_rb;
extern OSQPWorkspace workspace_compute_Fd_pair;

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
  bool success;
};


#ifdef CRAZYFLIE_FW

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "static_mem.h"
#include "eventtrigger.h"


#define CONTROLLER_LEE_PAYLOAD_QP_TASK_STACKSIZE (7 * configMINIMAL_STACK_SIZE)
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

// all times are in microseconds / 8 (i.e., up to 64*8ms can be measured)
EVENTTRIGGER(qpSolved, uint16, Fd, uint16, svm, uint16, mu, uint16, total)

#else

void print_csc_matrix(csc *M, const char *name)
{
  c_int j, i, row_start, row_stop;
  c_int k = 0;

  // Print name
  printf("%s :\n", name);

  for (j = 0; j < M->n; j++) {
    row_start = M->p[j];
    row_stop  = M->p[j + 1];

    if (row_start == row_stop) continue;
    else {
      for (i = row_start; i < row_stop; i++) {
        printf("\t[%3u,%3u] = %.3g\n", (int)M->i[i], (int)j, M->x[k++]);
      }
    }
  }
}

void print_dns_matrix(c_float *M, c_int m, c_int n, const char *name)
{
  c_int i, j;

  printf("%s : \n\t", name);

  for (i = 0; i < m; i++) {   // Cycle over rows
    for (j = 0; j < n; j++) { // Cycle over columns
      if (j < n - 1)
        // c_print("% 14.12e,  ", M[j*m+i]);
        printf("% .3g,  ", M[j * m + i]);

      else
        // c_print("% 14.12e;  ", M[j*m+i]);
        printf("% .3g;  ", M[j * m + i]);
    }

    if (i < m - 1) {
      printf("\n\t");
    }
  }
  printf("\n");
}

void print_vec(c_float *v, c_int n, const char *name) {
  print_dns_matrix(v, 1, n, name);
}

#endif

// helper function similar as first part of osqp_update_A and osqp_update_P
// to make updating entries more time and memory efficient
static void osqp_prepare_update(OSQPWorkspace *work)
{
  if (work->settings->scaling) {
    // Unscale data
    unscale_data(work);
  }
}

// helper function similar as first part of osqp_update_A and osqp_update_P
// to make updating entries more time and memory efficient
static c_int osqp_finalize_update(OSQPWorkspace *work)
{
  c_int exitflag; // Exit flag

  if (work->settings->scaling) {
    // Scale data
    scale_data(work);
  }

  // Update linear system structure with new data
  exitflag = work->linsys_solver->update_matrices(work->linsys_solver,
                                                  work->data->P,
                                                  work->data->A);

  // Reset solver information
  reset_info(work->info);

  return exitflag;
}

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

static inline void computePlaneNormals(struct vec p1, struct vec p2, struct vec pload, float r, float l1, float l2, float lambda_svm, struct vec Fd, struct vec* n1, struct vec* n2,
  struct osqp_warmstart_hyperplane* warmstart_state) {
  // relative coordinates
  struct vec p1_r = vsub(p1, pload);
  struct vec p2_r = vsub(p2, pload);

  // run QP to find separating hyperplane
  OSQPWorkspace* workspace = &workspace_hyperplane;

  osqp_prepare_update(workspace);

  // Update P
  workspace->data->P->x[3] = 2 * lambda_svm;

  // Update A
  workspace->data->A->x[0] = Fd.x;
  workspace->data->A->x[1] = p1_r.x;
  workspace->data->A->x[2] = -p2_r.x;
  workspace->data->A->x[3] = Fd.y;
  workspace->data->A->x[4] = p1_r.y;
  workspace->data->A->x[5] = -p2_r.y;
  workspace->data->A->x[6] = Fd.z;
  workspace->data->A->x[7] = p1_r.z;
  workspace->data->A->x[8] = -p2_r.z;

  osqp_finalize_update(workspace);

  osqp_warm_start(workspace, warmstart_state->x, warmstart_state->y);

  #ifdef CRAZYFLIE_FW
    uint64_t timestamp_svm_start = usecTimestamp();
  #endif
  osqp_solve(workspace);
  #ifdef CRAZYFLIE_FW
    eventTrigger_qpSolved_payload.svm += (usecTimestamp() - timestamp_svm_start) / 8;
  #endif

  if (workspace->info->status_val == OSQP_SOLVED) {
    memcpy(warmstart_state->x, workspace->solution->x, sizeof(warmstart_state->x));
    memcpy(warmstart_state->y, workspace->solution->y, sizeof(warmstart_state->y));

    struct vec n = vloadf(workspace->solution->x);

    // now rotate the hyperplane in two directions to compute the two resulting hyperplanes
    struct vec axis = vcross(n, mkvec(0,0,1));

    if (r > 2.0f * l1) {
      *n1 = n;
    } else {
      float angle1 = 2 * asinf(r / (2.0f * l1));
      struct quat q1 = qaxisangle(axis, angle1);
      *n1 = qvrot(q1, n);
    }

    if (r > 2.0f * l2) {
      *n2 = n;
    } else {
      float angle2 = 2 * asinf(r / (2.0f * l2));
      struct quat q2 = qaxisangle(axis, -angle2);
      *n2 = vneg(qvrot(q2, n));
    }
  } else {
  #ifdef CRAZYFLIE_FW
        DEBUG_PRINT("QPsvm: %s\n", workspace->info->status);
  #else
        printf("QPsvm: %s\n", workspace->info->status);
  #endif

    *n1 = vzero();
    *n2 = vzero();
  }
}

// Computes the circle that is created when intersecting a plane given as (n.p = a)
// with a sphere at center c with radius r
// See https://math.stackexchange.com/questions/943383/determine-circle-of-intersection-of-plane-and-sphere
static void plane_sphere_intersection(struct vec n, float a, struct vec c, float r, struct vec* result_center, float* result_radius)
{
  // compute signed distance sphere -> hyperplane
  float length = vmag(n);
  float dist = (vdot(n, c) - a) / length;

  // if the minimum distance is greater than r, there is no intersection
  if (fabsf(dist) > r) {
    *result_center = vzero();
    *result_radius = -1;
    return;
  }
  // otherwise, compute the center point (on the plane) and radius of the resulting circle
  *result_center = vsub(c, vscl(dist / length, n));
  *result_radius = sqrtf(r*r - dist*dist);
}

// projects point p onto plane (given by n.p = a)
static struct vec project_point_on_plane(struct vec n, float a, struct vec p)
{
    // compute signed distance sphere -> hyperplane
    float length = vmag(n);
    float dist = (vdot(n, p) - a) / length;

    struct vec p_on_plane = vsub(p, vscl(dist / length, n));
    return p_on_plane;
}

static inline void computePlaneNormals_rb(
  struct vec p1,
  struct vec p2,
  struct vec p1_attached,
  struct vec p2_attached,
  float r,
  float l1,
  float l2, float lambda_svm, struct vec Fd1, struct vec Fd2,
  struct vec* n1, struct vec* n2,
  struct osqp_warmstart_hyperplane_rb* warmstart_state) {

  // TODO: relative to payload to aid warm start!

  // run QP to find separating hyperplane
  OSQPWorkspace* workspace = &workspace_hyperplane_rb;

  osqp_prepare_update(workspace);

  struct vec Fd1t = vadd(p1_attached, Fd1);
  struct vec Fd2t = vadd(p2_attached, Fd2);

  // Update A
  workspace->data->A->x[0] = p1.x;
  workspace->data->A->x[1] = p1_attached.x;
  workspace->data->A->x[2] = -p2.x;
  workspace->data->A->x[3] = -p2_attached.x;
  workspace->data->A->x[4] = Fd1t.x;
  workspace->data->A->x[5] = -Fd2t.x;

  workspace->data->A->x[6] = p1.y;
  workspace->data->A->x[7] = p1_attached.y;
  workspace->data->A->x[8] = -p2.y;
  workspace->data->A->x[9] = -p2_attached.y;
  workspace->data->A->x[10] = Fd1t.y;
  workspace->data->A->x[11] = -Fd2t.y;

  workspace->data->A->x[12] = p1.z;
  workspace->data->A->x[13] = p1_attached.z;
  workspace->data->A->x[14] = -p2.z;
  workspace->data->A->x[15] = -p2_attached.z;
  workspace->data->A->x[16] = Fd1t.z;
  workspace->data->A->x[17] = -Fd2t.z;

  // print_csc_matrix(workspace->data->P, "P");
  // print_csc_matrix(workspace->data->A, "A");
  // print_vec(workspace->data->q, workspace->data->n, "q");
  // print_vec(workspace->data->u, workspace->data->n, "u");
  // print_vec(workspace->data->l, workspace->data->n, "l");

  osqp_finalize_update(workspace);

  c_float q_new[6] = {0, 0, 0, lambda_svm, lambda_svm, 0 };

  osqp_update_lin_cost(workspace, q_new);

  osqp_warm_start(workspace, warmstart_state->x, warmstart_state->y);

  #ifdef CRAZYFLIE_FW
    uint64_t timestamp_svm_start = usecTimestamp();
  #endif
  osqp_solve(workspace);
  #ifdef CRAZYFLIE_FW
    eventTrigger_qpSolved_payload.svm += (usecTimestamp() - timestamp_svm_start) / 8;
  #endif

  if (workspace->info->status_val == OSQP_SOLVED/* || workspace->info->status_val == OSQP_SOLVED_INACCURATE || workspace->info->status_val == OSQP_MAX_ITER_REACHED*/) {
    memcpy(warmstart_state->x, workspace->solution->x, sizeof(warmstart_state->x));
    memcpy(warmstart_state->y, workspace->solution->y, sizeof(warmstart_state->y));

    struct vec n = vloadf(workspace->solution->x);
    // the QP adds additional decision variables...
    float a = workspace->solution->x[5];

    // static int counter = 0;
    // ++counter;
    // if (counter % 100 == 0) {
    //   DEBUG_PRINT("hp %f %f %f %f\n", (double)n.x, (double)n.y, (double)n.z, (double)a);

    // }

    // For UAV1, compute the resulting intersection plane
    struct vec center;
    float radius;
    plane_sphere_intersection(n, a, p1_attached, l1, &center, &radius);

    // if (counter % 100 == 0) {
    //   DEBUG_PRINT("int %f %f %f %f\n", (double)center.x, (double)center.y, (double)center.z, (double)radius);

    // }

    if (radius >= 0) {
      // compute the intersection point with the highest z-value
      struct vec point_on_plane = project_point_on_plane(n, a, vadd(center, mkvec(0,0,1)));
      struct vec dir = vnormalize(vsub(point_on_plane, center));
      struct vec intersection_high_z = vadd(center, vscl(radius, dir));

      // the updated plane connects three points (where p1 == new relative coordinate system)
      struct vec point0 = p1_attached;
      struct vec point1 = vadd(p1_attached, vcross(n, mkvec(0,0,1)));
      struct vec point2 = intersection_high_z;

      struct vec n1_untilted = vcross(vsub(point0, point1), vsub(point0, point2));

      // now rotate the hyperplane to compute the two resulting hyperplane
      struct vec axis = vcross(n, mkvec(0,0,1));

      if (r > 2.0f * l1) {
        *n1 = n1_untilted;
      } else {
        float angle1 = 2 * asinf(r / (2.0f * l1));
        struct quat q1 = qaxisangle(axis, angle1);
        *n1 = vneg(qvrot(q1, n1_untilted));
      }
    } else {
      //printf("no int 1\n");
      // the plane is so far, that we don't need a safety hyperplane!
      *n1 = vzero();
    }

    // For UAV2, compute the resulting intersection plane
    plane_sphere_intersection(n, a, p2_attached, l2, &center, &radius);

    if (radius >= 0) {
      // compute the intersection point with the highest z-value
      struct vec point_on_plane = project_point_on_plane(n, a, vadd(center, mkvec(0,0,1)));
      struct vec dir = vnormalize(vsub(point_on_plane, center));
      struct vec intersection_high_z = vadd(center, vscl(radius, dir));

      // the updated plane connects three points (where p2 == new relative coordinate system)
      struct vec point0 = p2_attached;
      struct vec point1 = vadd(p2_attached, vcross(n, mkvec(0,0,1)));
      struct vec point2 = intersection_high_z;

      struct vec n2_untilted = vcross(vsub(point0, point1), vsub(point0, point2));

      // now rotate the hyperplane to compute the two resulting hyperplane
      struct vec axis = vcross(n, mkvec(0,0,1));

      if (r > 2.0f * l1) {
        *n2 = n2_untilted;
      } else {
        float angle1 = 2 * asinf(r / (2.0f * l1));
        struct quat q1 = qaxisangle(axis, -angle1);
        *n2 = qvrot(q1, n2_untilted);
      }
    } else {
      // printf("no int 2 %f %f %f %f\n", p2_attached.x, p2_attached.y, p2_attached.z, l2);
      // printf("no int 2b %f %f %f %f\n", n.x, n.y, n.z, a);

      // the plane is so far, that we don't need a safety hyperplane!
      *n2 = vzero();
    }

  } else {
  #ifdef CRAZYFLIE_FW
        DEBUG_PRINT("QPsvm: %s\n", workspace->info->status);
        DEBUG_PRINT("QPsvm: %s %d %f\n", workspace->info->status, workspace->info->iter, (double)workspace->info->obj_val);
        DEBUG_PRINT("p1 = %f, %f, %f\n",(double) p1.x, (double)p1.y, (double)p1.z);
        DEBUG_PRINT("p2 = %f, %f, %f\n", (double)p2.x, (double)p2.y, (double)p2.z);
        DEBUG_PRINT("p1a = %f, %f, %f\n", (double)p1_attached.x, (double)p1_attached.y, (double)p1_attached.z);
        DEBUG_PRINT("p2a = %f, %f, %f\n", (double)p2_attached.x, (double)p2_attached.y, (double)p2_attached.z);
        DEBUG_PRINT("Fd1t = %f, %f, %f\n", (double)Fd1t.x, (double)Fd1t.y, (double)Fd1t.z);
        DEBUG_PRINT("Fd2t = %f, %f, %f\n", (double)Fd2t.x, (double)Fd2t.y, (double)Fd2t.z);
        DEBUG_PRINT("lambda_svm = %f\n", (double)lambda_svm);
  #else
        printf("QPsvm: %s %d %f\n", workspace->info->status, workspace->info->iter, workspace->info->obj_val);
        printf("p1 = %f, %f, %f\n", p1.x, p1.y, p1.z);
        printf("p2 = %f, %f, %f\n", p2.x, p2.y, p2.z);
        printf("p1a = %f, %f, %f\n", p1_attached.x, p1_attached.y, p1_attached.z);
        printf("p2a = %f, %f, %f\n", p2_attached.x, p2_attached.y, p2_attached.z);
        printf("Fd1t = %f, %f, %f\n", Fd1t.x, Fd1t.y, Fd1t.z);
        printf("Fd2t = %f, %f, %f\n", Fd2t.x, Fd2t.y, Fd2t.z);
        printf("lambda_svm = %f\n", lambda_svm);
  #endif

    *n1 = vzero();
    *n2 = vzero();
  }
}

static bool compute_Fd_pair_qp(struct quat payload_quat, struct vec attPoint1, struct vec attPoint2, struct vec F_d, struct vec M_d, struct vec* F_d1, struct vec* F_d2, struct osqp_warmstart_compute_Fd_pair* warmstart_state)
{
  // printf("\nFd: %f %f %f\n", F_d.x, F_d.y, F_d.z);
  // printf("Md: %f %f %f\n", M_d.x, M_d.y, M_d.z);
  // printf("q: %f %f %f %f\n", payload_quat.x, payload_quat.y, payload_quat.z, payload_quat.w);
  // printf("ap1: %f %f %f\n", attPoint1.x, attPoint1.y, attPoint1.z);
  // printf("ap2: %f %f %f\n", attPoint2.x, attPoint2.y, attPoint2.z);

  struct mat33 R0t = mtranspose(quat2rotmat(payload_quat));

  struct mat33 attPR0t = mmul(mcrossmat(attPoint1), R0t);
  struct mat33 attP2R0t = mmul(mcrossmat(attPoint2), R0t);

  OSQPWorkspace* workspace = &workspace_compute_Fd_pair;

  osqp_prepare_update(workspace);

  // Update A
  c_float* x = workspace->data->A->x;
  x[ 0] = attPR0t.m[0][0];
  x[ 1] = attPR0t.m[1][0];
  x[ 2] = attPR0t.m[2][0];

  x[ 4] = attPR0t.m[0][1];
  x[ 5] = attPR0t.m[1][1];
  x[ 6] = attPR0t.m[2][1];

  x[ 8] = attPR0t.m[0][2];
  x[ 9] = attPR0t.m[1][2];
  x[10] = attPR0t.m[2][2];

  x[12] = attP2R0t.m[0][0];
  x[13] = attP2R0t.m[1][0];
  x[14] = attP2R0t.m[2][0];

  x[16] = attP2R0t.m[0][1];
  x[17] = attP2R0t.m[1][1];
  x[18] = attP2R0t.m[2][1];

  x[20] = attP2R0t.m[0][2];
  x[21] = attP2R0t.m[1][2];
  x[22] = attP2R0t.m[2][2];


  osqp_finalize_update(workspace);

  // update q, l, and u (after finalize update!)
  c_float l_new[6] =  {M_d.x, M_d.y, M_d.z, F_d.x, F_d.y, F_d.z};
  c_float u_new[6] =  {M_d.x, M_d.y, M_d.z, F_d.x, F_d.y, F_d.z};

  osqp_update_lower_bound(workspace, l_new);
  osqp_update_upper_bound(workspace, u_new);

  osqp_warm_start(workspace, warmstart_state->x, warmstart_state->y);

  #ifdef CRAZYFLIE_FW
    uint64_t timestamp_Fd_start = usecTimestamp();
  #endif
  osqp_solve(workspace);
  #ifdef CRAZYFLIE_FW
    eventTrigger_qpSolved_payload.Fd += (usecTimestamp() - timestamp_Fd_start) / 8;
  #endif

  if (workspace->info->status_val == OSQP_SOLVED) {
    memcpy(warmstart_state->x, workspace->solution->x, sizeof(warmstart_state->x));
    memcpy(warmstart_state->y, workspace->solution->y, sizeof(warmstart_state->y));

    F_d1->x = (workspace)->solution->x[0];
    F_d1->y = (workspace)->solution->x[1];
    F_d1->z = (workspace)->solution->x[2];
    F_d2->x =  (workspace)->solution->x[3];
    F_d2->y =  (workspace)->solution->x[4];
    F_d2->z =  (workspace)->solution->x[5];

    // printf("Fd1: %f %f %f\n", F_d1->x, F_d1->y, F_d1->z);
    // printf("Fd2: %f %f %f\n", F_d2->x, F_d2->y, F_d2->z);



    return true;
  } else {
  #ifdef CRAZYFLIE_FW
        DEBUG_PRINT("QPp: %s\n", workspace->info->status);
  #else
        printf("QPp: %s\n", workspace->info->status);
  #endif
  }
  return false;
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
  .Kprot_I = {0.0, 0.0, 0.0},
  .Kprot_I_limit = 100,

  // Cables PD
  .K_q = {25, 25, 25},
  .K_q_limit = 100,
  .K_w = {24, 24, 24},
  .K_w_limit = 100,
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
  .formation_control = 0,

  .radius = 0.15,

  .lambdaa = 0.0,
  .lambda_svm = 0.0,
  .gen_hp = 0,

  // automatically compute by default
  .attachement_points[0].l = -1,
  .attachement_points[1].l = -1,
  .attachement_points[2].l = -1,

  .Pinvs[0].Pinv.m = {{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0}},
  .Pinvs[1].Pinv.m = {{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0}},
  .Pinvs[2].Pinv.m = {{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0}},
};

// static inline struct vec vclampscl(struct vec value, float min, float max) {
//   return mkvec(
//     clamp(value.x, min, max),
//     clamp(value.y, min, max),
//     clamp(value.z, min, max));
// }

static void runQP(const struct QPInput *input, struct QPOutput* output)
{
#ifdef CRAZYFLIE_FW
  eventTrigger_qpSolved_payload.Fd = 0;
  eventTrigger_qpSolved_payload.svm = 0;
  eventTrigger_qpSolved_payload.mu = 0;
  uint64_t timestamp_total_start = usecTimestamp();
#endif

  struct vec F_d = input->F_d;
  struct vec M_d = input->M_d;
  uint8_t num_neighbors = input->num_neighbors;
  struct vec statePos = input->statePos;
  struct vec plStPos = input->plStPos;
  // struct quat plStquat = input->plStquat;
  float radius = input->self->radius;
  struct vec desVirtInp   = input->self->desVirtInp;
  struct vec desVirt_prev = input->self->desVirtInp;

  output->timestamp = input->timestamp;
  output->success = false;

  bool is_rigid_body = !isnanf(input->plStquat.w);
  
  if (num_neighbors >= 1) {
   // declare additional variables
    struct vec statePos2 = input->statePos2;
    struct vec attPoint = input->self->attachement_points[0].point;
    struct vec attPoint2 = input->self->attachement_points[1].point;
    struct vec desVirt2_prev = input->self->desVirt2_prev;

    struct vec muDes = input->self->attachement_points[0].mu_desired;
    struct vec muDes2 = input->self->attachement_points[1].mu_desired;

    if (num_neighbors == 1) {

      float l1 = -1;
      float l2 = -1;
      // Set corresponding attachment points
      for (uint8_t i = 0; i < num_neighbors+1; ++i) {
        if (input->self->attachement_points[i].id == input->ids[0]) {
          attPoint2 = input->self->attachement_points[i].point;
          muDes2 = input->self->attachement_points[i].mu_desired;
          l2 = input->self->attachement_points[i].l;
        } else {
          attPoint = input->self->attachement_points[i].point;
          muDes = input->self->attachement_points[i].mu_desired;
          l1 = input->self->attachement_points[i].l;
        }
      }

      if (is_rigid_body) {
        // QP for 2 uavs 1 hp, rigid rod will be added it here!
        struct vec plSt_att = vadd(plStPos, qvrot(input->plStquat, attPoint));
        struct vec plSt_att2 = vadd(plStPos, qvrot(input->plStquat, attPoint2));
        // automatically compute cable length, if desired
        if (l1 <= 0) {
          l1 = vmag(vsub(plSt_att, statePos));
        }
        if (l2 <= 0) {
          l2 = vmag(vsub(plSt_att2, statePos2));
        }

        // static int counter = 0;
        // if (counter % 100 == 0) {
        //   DEBUG_PRINT("alqp1 %f %f %f %f\n", (double)attPoint.x, (double)attPoint.y, (double)attPoint.z, (double)l1); 
        //   DEBUG_PRINT("alqp1 %f %f %f %f\n", (double)attPoint2.x, (double)attPoint2.y, (double)attPoint2.z, (double)l2); 
        // }
        // ++counter;

        // no control over pitch -> set y component to zero
        M_d.y = 0;

        struct vec n1;
        struct vec n2;
        if (input->self->gen_hp == 0) {
          n1 = computePlaneNormal(statePos, statePos2, plSt_att, radius, l1, l2);
          n2 = computePlaneNormal(statePos2, statePos, plSt_att2, radius, l2, l1);
        } else {
          struct vec Fd1;
          struct vec Fd2;
          // basic method
          if (input->self->gen_hp == 1) {
            Fd1 = vscl(0.5f, F_d);
            Fd2 = vscl(0.5f, F_d);
          }
          else if (input->self->gen_hp == 2) {
            // advanced method
            // F_d = mkvec(0,0.1,0.5);
            // M_d = mkvec(0.5,0,0);
            struct mat33 Rp = quat2rotmat(input->plStquat);
            struct mat66 R_blockdiag = zero66();
            for (int i = 0; i < 3; ++i) {
              for (int j = 0; j < 3; ++j) {
                R_blockdiag.m[i][j] = Rp.m[i][j];
                R_blockdiag.m[3+i][3+j] = Rp.m[i][j];
              }
            }
            struct vec Fdr = mvmul(mtranspose(Rp), F_d);
            struct mat66 R_blockdiag_times_Pinv= mmul66(R_blockdiag, input->self->Pinvs[0].Pinv);

            float v[6] = {Fdr.x, Fdr.y, Fdr.z, M_d.x, M_d.y, M_d.z};
            Fd1 = vzero();
            Fd2 = vzero();
            for (int j=0; j < 6; ++j) {
              Fd1.x += R_blockdiag_times_Pinv.m[0][j] * v[j];
              Fd1.y += R_blockdiag_times_Pinv.m[1][j] * v[j];
              Fd1.z += R_blockdiag_times_Pinv.m[2][j] * v[j];
              Fd2.x += R_blockdiag_times_Pinv.m[3][j] * v[j];
              Fd2.y += R_blockdiag_times_Pinv.m[4][j] * v[j];
              Fd2.z += R_blockdiag_times_Pinv.m[5][j] * v[j];
            }


            if (input->self->Pinvs[0].id1 == input->ids[0]) {
              // Pinv was with respect to other robot -> swap result
              struct vec tmp;
              tmp = Fd2;
              Fd2 = Fd1;
              Fd1 = tmp;
            }

            // printf("Fd1: %f %f %f\n", Fd1.x, Fd1.y, Fd1.z);
            // printf("Fd2: %f %f %f\n", Fd2.x, Fd2.y, Fd2.z);
          } else if (input->self->gen_hp == 3) {
            compute_Fd_pair_qp(input->plStquat, attPoint, attPoint2, F_d, M_d, &Fd1, &Fd2, &input->self->osqp_warmstart_compute_Fd_pairs[0]);
            // printf("Fd1: %f %f %f\n", Fd1.x, Fd1.y, Fd1.z);
            // printf("Fd2: %f %f %f\n", Fd2.x, Fd2.y, Fd2.z);
          }

          // static int counter = 0;
          // if (counter % 100 == 0) {
          // DEBUG_PRINT("\nFd: %f %f %f\n", (double)F_d.x, (double)F_d.y, (double)F_d.z);
          // DEBUG_PRINT("Md: %f %f %f\n", (double)M_d.x, (double)M_d.y, (double)M_d.z);
          // DEBUG_PRINT("Fd1: %f %f %f\n", (double)Fd1.x, (double)Fd1.y, (double)Fd1.z);
          // DEBUG_PRINT("Fd2: %f %f %f\n", (double)Fd2.x, (double)Fd2.y, (double)Fd2.z);
          // }
          // ++counter;
          computePlaneNormals_rb(statePos, statePos2, plSt_att, plSt_att2, radius, l1, l2, input->self->lambda_svm, Fd1, Fd2, &n1, &n2, &input->self->osqp_warmstart_hyperplane_rbs[0]);
        }
        
        struct mat33 R0t = mtranspose(quat2rotmat(input->plStquat));

        struct mat33 attPR0t = mmul(mcrossmat(attPoint), R0t);
        struct mat33 attP2R0t = mmul(mcrossmat(attPoint2), R0t);

        // static int counter = 0;
        // if (counter % 100 == 0) {
        //   DEBUG_PRINT("q %f %f %f %f\n", (double)input->plStquat.w, (double)input->plStquat.x, (double)input->plStquat.y, (double)input->plStquat.z);
        //   DEBUG_PRINT("attPoint %f %f %f\n", (double)attPoint.x, (double)attPoint.y, (double)attPoint.z);
        //   DEBUG_PRINT("attPoint2 %f %f %f\n", (double)attPoint2.x, (double)attPoint2.y, (double)attPoint2.z);
        // }
        // ++counter;

        OSQPWorkspace* workspace = &workspace_2uav_1hp_rod;
        workspace->settings->warm_start = 1;

        osqp_prepare_update(workspace);

        // Update A
        c_float* x = workspace->data->A->x;
        x[0 ] = R0t.m[0][0]; x[1 ] = R0t.m[1][0]; x[ 2] = R0t.m[2][0]; x[3 ] =  attPR0t.m[0][0]; x[4 ] =  attPR0t.m[1][0];  x[5 ] =  attPR0t.m[2][0]; x[6 ] = n1.x;
        x[7 ] = R0t.m[0][1]; x[8 ] = R0t.m[1][1]; x[ 9] = R0t.m[2][1]; x[10] =  attPR0t.m[0][1]; x[11] =  attPR0t.m[1][1];  x[12] =  attPR0t.m[2][1]; x[13] = n1.y;
        x[14] = R0t.m[0][2]; x[15] = R0t.m[1][2]; x[16] = R0t.m[2][2]; x[17] =  attPR0t.m[0][2]; x[18] =  attPR0t.m[1][2];  x[19] =  attPR0t.m[2][2]; x[20] = n1.z;

        x[21] = R0t.m[0][0]; x[22] = R0t.m[1][0]; x[23] = R0t.m[2][0]; x[24] = attP2R0t.m[0][0]; x[25] = attP2R0t.m[1][0];  x[26] = attP2R0t.m[2][0]; x[27] = n2.x;
        x[28] = R0t.m[0][1]; x[29] = R0t.m[1][1]; x[30] = R0t.m[2][1]; x[31] = attP2R0t.m[0][1]; x[32] = attP2R0t.m[1][1];  x[33] = attP2R0t.m[2][1]; x[34] = n2.y;
        x[35] = R0t.m[0][2]; x[36] = R0t.m[1][2]; x[37] = R0t.m[2][2]; x[38] = attP2R0t.m[0][2]; x[39] = attP2R0t.m[1][2];  x[40] = attP2R0t.m[2][2]; x[41] = n2.z;

        // print_csc_matrix(workspace->data->A, "A");

        osqp_finalize_update(workspace);

        // update q, l, and u (after finalize update!)
        struct vec F_dP = mvmul(R0t, F_d);
        c_float l_new[8] =  {F_dP.x,	F_dP.y,	F_dP.z, M_d.x, M_d.y, M_d.z, -INFINITY, -INFINITY,};
        c_float u_new[8] =  {F_dP.x,	F_dP.y,	F_dP.z, M_d.x, M_d.y, M_d.z, 0, 0,};

        /* P = np.eye(9)
          1/2 x^2 + lambda (x^2 - 2xx_d + x_d^2)
          => J = (1/2+lambda) x^2 - 2 * lambda x_d x
          =>     1/2 x^2 - 1 * lambda / (1/2+lambda) x_d x
          => q = -2 * lambda / (1+2*lambda) * x_d
        */
        const float factor = - 2.0f * input->self->lambdaa / (1.0f + 2.0f * input->self->lambdaa);
        c_float q_new[6];

        if (input->self->formation_control == 0) {
          q_new[0] = 0.0f;
          q_new[1] = 0.0f;
          q_new[2] = 0.0f;
          q_new[3] = 0.0f;
          q_new[4] = 0.0f;
          q_new[5] = 0.0f;
        }
        else if (input->self->formation_control == 1) {
          q_new[0] = factor * desVirt_prev.x;
          q_new[1] = factor * desVirt_prev.y;
          q_new[2] = factor * desVirt_prev.z;
          q_new[3] = factor * desVirt2_prev.x;
          q_new[4] = factor * desVirt2_prev.y;
          q_new[5] = factor * desVirt2_prev.z;
        } else {
          float scale = vmag(F_d) / 2.0f;
          muDes = vsclnorm(muDes, scale);
          muDes2 = vsclnorm(muDes2, scale);

          q_new[0] = factor * muDes.x;
          q_new[1] = factor * muDes.y;
          q_new[2] = factor * muDes.z;
          q_new[3] = factor * muDes2.x;
          q_new[4] = factor * muDes2.y;
          q_new[5] = factor * muDes2.z;
        }

        // DEBUG_PRINT("qn %f %f %f %f %f %f\n", (double)q_new[0], (double)q_new[1], (double)q_new[2], (double)q_new[3], (double)q_new[4], (double)q_new[5]);

        
        osqp_update_lin_cost(workspace, q_new);
        osqp_update_lower_bound(workspace, l_new);
        osqp_update_upper_bound(workspace, u_new);

        #ifdef CRAZYFLIE_FW
          uint64_t timestamp_mu_start = usecTimestamp();
        #endif
        osqp_solve(workspace);
        #ifdef CRAZYFLIE_FW
          eventTrigger_qpSolved_payload.mu += (usecTimestamp() - timestamp_mu_start) / 8;
        #endif

        if (workspace->info->status_val == OSQP_SOLVED) {
          desVirtInp.x = (workspace)->solution->x[0];
          desVirtInp.y = (workspace)->solution->x[1];
          desVirtInp.z = (workspace)->solution->x[2];
          input->self->desVirt2_prev.x =  (workspace)->solution->x[3];
          input->self->desVirt2_prev.y =  (workspace)->solution->x[4];
          input->self->desVirt2_prev.z =  (workspace)->solution->x[5];

          input->self->desVirtInp2 = input->self->desVirt2_prev;

          // // struct vec sanity_check = vsub(vadd(mvmul(attPR0t, desVirtInp), mvmul(attP2R0t, input->self->desVirt2_prev)), M_d);
          
          // static int counter = 0;
          // if (counter % 100 == 0) {
          //   DEBUG_PRINT("q %f %f %f %f\n", (double)input->plStquat.w, (double)input->plStquat.x, (double)input->plStquat.y, (double)input->plStquat.z);
          //   DEBUG_PRINT("mu1 %f %f %f\n", (double)desVirtInp.x, (double)desVirtInp.y, (double)desVirtInp.z);
          //   // struct vec Md1 = mvmul(attPR0t, desVirtInp);
          //   // DEBUG_PRINT("Md1 %f %f %f\n", (double)Md1.x, (double)Md1.y, (double)Md1.z);

          //   DEBUG_PRINT("mu2 %f %f %f\n", (double)input->self->desVirt2_prev.x, (double)input->self->desVirt2_prev.y, (double)input->self->desVirt2_prev.z);
          //   // DEBUG_PRINT("Mds %f %f %f\n", (double)sanity_check.x, (double)sanity_check.y, (double)sanity_check.z);
          // }
          // ++counter;

          output->success = true;
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
        // automatically compute cable length, if desired
        if (l1 <= 0) {
          l1 = vmag(vsub(plStPos, statePos));
        }
        if (l2 <= 0) {
          l2 = vmag(vsub(plStPos, statePos2));
        }
        // Solve QP for 2 uavs 1 hp point mass
        OSQPWorkspace* workspace = &workspace_2uav_2hp;
        workspace->settings->warm_start = 1;

        struct vec n1;
        struct vec n2;
        if (input->self->gen_hp == 0) {
          n1 = computePlaneNormal(statePos, statePos2, plStPos, radius, l1, l2);
          n2 = computePlaneNormal(statePos2, statePos, plStPos, radius, l2, l1);
        } else {
          computePlaneNormals(statePos, statePos2, plStPos, radius, l1, l2, input->self->lambda_svm, F_d, &n1, &n2, &input->self->osqp_warmstart_hyperplanes[0]);
        }

        osqp_prepare_update(workspace);

        // Update A
        workspace->data->A->x[1] = n1.x;
        workspace->data->A->x[3] = n1.y;
        workspace->data->A->x[5] = n1.z;
        workspace->data->A->x[7] = n2.x;
        workspace->data->A->x[9] = n2.y;
        workspace->data->A->x[11] = n2.z;

        osqp_finalize_update(workspace);

        // update q, l, and u (after finalize update!)
        c_float l_new[6] =  {F_d.x,	F_d.y,	F_d.z, -INFINITY, -INFINITY,};
        c_float u_new[6] =  {F_d.x,	F_d.y,	F_d.z, 0, 0,};

        /* P = np.eye(9)
            1/2 x^2 + lambda (x^2 - 2xx_d + x_d^2)
            => J = (1/2+lambda) x^2 - 2 * lambda x_d x
            =>     1/2 x^2 - 1 * lambda / (1/2+lambda) x_d x
            => q = -2 * lambda / (1+2*lambda) * x_d
        */
        const float factor = - 2.0f * input->self->lambdaa / (1.0f + 2.0f * input->self->lambdaa);
        c_float q_new[6];

        if (input->self->formation_control == 0) {
          q_new[0] = 0.0f;
          q_new[1] = 0.0f;
          q_new[2] = 0.0f;
          q_new[3] = 0.0f;
          q_new[4] = 0.0f;
          q_new[5] = 0.0f;
        }
        else if (input->self->formation_control == 1) {
          q_new[0] = factor * desVirt_prev.x;
          q_new[1] = factor * desVirt_prev.y;
          q_new[2] = factor * desVirt_prev.z;
          q_new[3] = factor * desVirt2_prev.x;
          q_new[4] = factor * desVirt2_prev.y;
          q_new[5] = factor * desVirt2_prev.z;
        } else {
          float scale = vmag(F_d) / 2.0f;
          muDes = vsclnorm(muDes, scale);
          muDes2 = vsclnorm(muDes2, scale);

          q_new[0] = factor * muDes.x;
          q_new[1] = factor * muDes.y;
          q_new[2] = factor * muDes.z;
          q_new[3] = factor * muDes2.x;
          q_new[4] = factor * muDes2.y;
          q_new[5] = factor * muDes2.z;
        }

        osqp_update_lin_cost(workspace, q_new);
        osqp_update_lower_bound(workspace, l_new);
        osqp_update_upper_bound(workspace, u_new);

        #ifdef CRAZYFLIE_FW
          uint64_t timestamp_mu_start = usecTimestamp();
        #endif
        osqp_solve(workspace);
        #ifdef CRAZYFLIE_FW
          eventTrigger_qpSolved_payload.mu += (usecTimestamp() - timestamp_mu_start) / 8;
        #endif

        if (workspace->info->status_val == OSQP_SOLVED) {
          desVirtInp.x = (workspace)->solution->x[0];
          desVirtInp.y = (workspace)->solution->x[1];
          desVirtInp.z = (workspace)->solution->x[2];
          input->self->desVirt2_prev.x =  (workspace)->solution->x[3];
          input->self->desVirt2_prev.y =  (workspace)->solution->x[4];
          input->self->desVirt2_prev.z =  (workspace)->solution->x[5];
          output->success = true;
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
      struct vec attPoint3 = input->self->attachement_points[2].point;  
      struct vec desVirt3_prev = input->self->desVirt3_prev;

      struct vec muDes3 = input->self->attachement_points[2].mu_desired;

      float l1 = -1;
      float l2 = -1;
      float l3 = -1;
      uint8_t myid = 0;
      // Set corresponding attachment points
      for (uint8_t i = 0; i < num_neighbors+1; ++i) {
        if (input->self->attachement_points[i].id == input->ids[0]) {
          attPoint2 = input->self->attachement_points[i].point;
          muDes2 = input->self->attachement_points[i].mu_desired;
          l2 = input->self->attachement_points[i].l;
        } else if (input->self->attachement_points[i].id == input->ids[1]) {
          attPoint3 = input->self->attachement_points[i].point;
          muDes3 = input->self->attachement_points[i].mu_desired;
          l3 = input->self->attachement_points[i].l;
        } else {
          attPoint = input->self->attachement_points[i].point;
          muDes = input->self->attachement_points[i].mu_desired;
          l1 = input->self->attachement_points[i].l;
          myid = input->self->attachement_points[i].id;
        }
      }

      if (num_neighbors == 2) {
        if (is_rigid_body) {
          // solve QP for 3 uavs 2 hps and rigid triangle payload
          OSQPWorkspace* workspace = &workspace_3uav_2hp_rig;
          workspace->settings->warm_start = 1;

          // c_float x_warm[9] = {  desVirt_prev.x,    desVirt_prev.y,    desVirt_prev.z,
          //                       desVirt2_prev.x,   desVirt2_prev.y,   desVirt2_prev.z, 
          //                       desVirt3_prev.x,   desVirt3_prev.y,   desVirt3_prev.z};
          // osqp_warm_start_x(workspace, x_warm);
          struct vec plSt_att = vadd(plStPos, qvrot(input->plStquat, attPoint));
          struct vec plSt_att2 = vadd(plStPos, qvrot(input->plStquat, attPoint2));
          struct vec plSt_att3 = vadd(plStPos, qvrot(input->plStquat, attPoint3));
          
          // automatically compute cable length, if desired
          if (l1 <= 0) {
            l1 = vmag(vsub(plSt_att, statePos));
          }
          if (l2 <= 0) {
            l2 = vmag(vsub(plSt_att2, statePos2));
          }
          if (l3 <= 0) {
            l3 = vmag(vsub(plSt_att3, statePos3));
          }
          struct vec n1, n2, n3, n4, n5, n6;
          if (input->self->gen_hp == 0) {
            n1 = computePlaneNormal(statePos, statePos2, plSt_att,  radius, l1, l2);
            n2 = computePlaneNormal(statePos, statePos3, plSt_att,  radius, l1, l3);
            n3 = computePlaneNormal(statePos2, statePos, plSt_att2,  radius, l2, l1);
            n4 = computePlaneNormal(statePos2, statePos3, plSt_att2, radius, l2, l3);
            n5 = computePlaneNormal(statePos3, statePos, plSt_att3,  radius, l3, l1);
            n6 = computePlaneNormal(statePos3, statePos2, plSt_att3, radius, l3, l2);
          } else if (input->self->gen_hp == 1) {
              struct vec Fd1 = vscl(0.5f, F_d);
              struct vec Fd2 = vscl(0.5f, F_d);        
              computePlaneNormals_rb(statePos,  statePos2, plSt_att, plSt_att2,  radius, l1, l2, input->self->lambda_svm, Fd1, Fd2, &n1, &n3, &input->self->osqp_warmstart_hyperplane_rbs[0]);
              computePlaneNormals_rb(statePos,  statePos3, plSt_att, plSt_att3,  radius, l1, l3, input->self->lambda_svm, Fd1, Fd2, &n2, &n5, &input->self->osqp_warmstart_hyperplane_rbs[1]);
              computePlaneNormals_rb(statePos2, statePos3, plSt_att2, plSt_att3, radius, l2, l3, input->self->lambda_svm, Fd1, Fd2, &n4, &n6, &input->self->osqp_warmstart_hyperplane_rbs[2]);
          } else if (input->self->gen_hp == 2) {
            struct vec Fd1, Fd2;

            // advanced method
            // F_d = mkvec(0,0.1,0.5);
            // M_d = mkvec(0.5,0,0);
            struct mat33 Rp = quat2rotmat(input->plStquat);
            struct mat66 R_blockdiag = zero66();
            for (int i = 0; i < 3; ++i) {
              for (int j = 0; j < 3; ++j) {
                R_blockdiag.m[i][j] = Rp.m[i][j];
                R_blockdiag.m[3+i][3+j] = Rp.m[i][j];
              }
            }
            struct vec Fdr = mvmul(mtranspose(Rp), F_d);

            // find the Pinvs index for 0 1
            int idx = -1;
            bool swap = false;
            for (int i = 0; i < 3; ++i) {
              if (input->self->Pinvs[i].id1 == myid && input->self->Pinvs[i].id2 == input->ids[0]) {
                idx = i;
                swap = false;
                break;
              }
              if (input->self->Pinvs[i].id2 == myid && input->self->Pinvs[i].id1 == input->ids[0]) {
                idx = i;
                swap = true;
                break;
              }
            }

            struct mat66 R_blockdiag_times_Pinv= mmul66(R_blockdiag, input->self->Pinvs[idx].Pinv);

            float v[6] = {Fdr.x, Fdr.y, Fdr.z, M_d.x, M_d.y, M_d.z};
            Fd1 = vzero();
            Fd2 = vzero();
            for (int j=0; j < 6; ++j) {
              Fd1.x += R_blockdiag_times_Pinv.m[0][j] * v[j];
              Fd1.y += R_blockdiag_times_Pinv.m[1][j] * v[j];
              Fd1.z += R_blockdiag_times_Pinv.m[2][j] * v[j];
              Fd2.x += R_blockdiag_times_Pinv.m[3][j] * v[j];
              Fd2.y += R_blockdiag_times_Pinv.m[4][j] * v[j];
              Fd2.z += R_blockdiag_times_Pinv.m[5][j] * v[j];
            }

            if (swap) {
              // Pinv was with respect to other robot -> swap result
              struct vec tmp;
              tmp = Fd2;
              Fd2 = Fd1;
              Fd1 = tmp;
            }

            computePlaneNormals_rb(statePos,  statePos2, plSt_att, plSt_att2,  radius, l1, l2, input->self->lambda_svm, Fd1, Fd2, &n1, &n3, &input->self->osqp_warmstart_hyperplane_rbs[0]);

            // find the Pinvs index for 0 2
            for (int i = 0; i < 3; ++i) {
              if (input->self->Pinvs[i].id1 == myid && input->self->Pinvs[i].id2 == input->ids[1]) {
                idx = i;
                swap = false;
                break;
              }
              if (input->self->Pinvs[i].id2 == myid && input->self->Pinvs[i].id1 == input->ids[1]) {
                idx = i;
                swap = true;
                break;
              }
            }

            R_blockdiag_times_Pinv= mmul66(R_blockdiag, input->self->Pinvs[idx].Pinv);
            Fd1 = vzero();
            Fd2 = vzero();
            for (int j=0; j < 6; ++j) {
              Fd1.x += R_blockdiag_times_Pinv.m[0][j] * v[j];
              Fd1.y += R_blockdiag_times_Pinv.m[1][j] * v[j];
              Fd1.z += R_blockdiag_times_Pinv.m[2][j] * v[j];
              Fd2.x += R_blockdiag_times_Pinv.m[3][j] * v[j];
              Fd2.y += R_blockdiag_times_Pinv.m[4][j] * v[j];
              Fd2.z += R_blockdiag_times_Pinv.m[5][j] * v[j];
            }

            if (swap) {
              // Pinv was with respect to other robot -> swap result
              struct vec tmp;
              tmp = Fd2;
              Fd2 = Fd1;
              Fd1 = tmp;
            }

            computePlaneNormals_rb(statePos,  statePos3, plSt_att, plSt_att3,  radius, l1, l3, input->self->lambda_svm, Fd1, Fd2, &n2, &n5, &input->self->osqp_warmstart_hyperplane_rbs[1]);

            // find the Pinvs index for 1 2
            for (int i = 0; i < 3; ++i) {
              if (input->self->Pinvs[i].id1 == input->ids[0] && input->self->Pinvs[i].id2 == input->ids[1]) {
                idx = i;
                swap = false;
                break;
              }
              if (input->self->Pinvs[i].id2 == input->ids[0] && input->self->Pinvs[i].id1 == input->ids[1]) {
                idx = i;
                swap = true;
                break;
              }
            }

            R_blockdiag_times_Pinv= mmul66(R_blockdiag, input->self->Pinvs[idx].Pinv);
            Fd1 = vzero();
            Fd2 = vzero();
            for (int j=0; j < 6; ++j) {
              Fd1.x += R_blockdiag_times_Pinv.m[0][j] * v[j];
              Fd1.y += R_blockdiag_times_Pinv.m[1][j] * v[j];
              Fd1.z += R_blockdiag_times_Pinv.m[2][j] * v[j];
              Fd2.x += R_blockdiag_times_Pinv.m[3][j] * v[j];
              Fd2.y += R_blockdiag_times_Pinv.m[4][j] * v[j];
              Fd2.z += R_blockdiag_times_Pinv.m[5][j] * v[j];
            }

            if (swap) {
              // Pinv was with respect to other robot -> swap result
              struct vec tmp;
              tmp = Fd2;
              Fd2 = Fd1;
              Fd1 = tmp;
            }

            computePlaneNormals_rb(statePos2, statePos3, plSt_att2, plSt_att3, radius, l2, l3, input->self->lambda_svm, Fd1, Fd2, &n4, &n6, &input->self->osqp_warmstart_hyperplane_rbs[2]);
          
          } else if (input->self->gen_hp == 3) {
            struct vec Fd1, Fd2;
            compute_Fd_pair_qp(input->plStquat, attPoint, attPoint2, F_d, M_d, &Fd1, &Fd2, &input->self->osqp_warmstart_compute_Fd_pairs[0]);
            computePlaneNormals_rb(statePos,  statePos2, plSt_att, plSt_att2,  radius, l1, l2, input->self->lambda_svm, Fd1, Fd2, &n1, &n3, &input->self->osqp_warmstart_hyperplane_rbs[0]);

            compute_Fd_pair_qp(input->plStquat, attPoint, attPoint3, F_d, M_d, &Fd1, &Fd2, &input->self->osqp_warmstart_compute_Fd_pairs[1]);
            computePlaneNormals_rb(statePos,  statePos3, plSt_att, plSt_att3,  radius, l1, l3, input->self->lambda_svm, Fd1, Fd2, &n2, &n5, &input->self->osqp_warmstart_hyperplane_rbs[1]);

            compute_Fd_pair_qp(input->plStquat, attPoint2, attPoint3, F_d, M_d, &Fd1, &Fd2, &input->self->osqp_warmstart_compute_Fd_pairs[2]);
            computePlaneNormals_rb(statePos2, statePos3, plSt_att2, plSt_att3, radius, l2, l3, input->self->lambda_svm, Fd1, Fd2, &n4, &n6, &input->self->osqp_warmstart_hyperplane_rbs[2]);
          }

          struct mat33 R0t = mtranspose(quat2rotmat(input->plStquat));

          struct mat33 attPR0t  = mmul(mcrossmat(attPoint),  R0t);
          struct mat33 attP2R0t = mmul(mcrossmat(attPoint2), R0t);
          struct mat33 attP3R0t = mmul(mcrossmat(attPoint3), R0t);

          osqp_prepare_update(workspace);

          // Update A
          c_float* x = workspace->data->A->x;
          x[0 ] = R0t.m[0][0];
          x[1 ] = R0t.m[1][0];
          x[2 ] = R0t.m[2][0];
          x[3 ] = attPR0t.m[0][0];
          x[4 ] = attPR0t.m[1][0];
          x[5 ] = attPR0t.m[2][0];
          x[6 ] = n1.x;
          x[7 ] = n2.x;
          x[8 ] = R0t.m[0][1];
          x[9 ] = R0t.m[1][1];
          x[10] = R0t.m[2][1];
          x[11] = attPR0t.m[0][1];
          x[12] = attPR0t.m[1][1];
          x[13] = attPR0t.m[2][1];
          x[14] = n1.y;
          x[15] = n2.y;
          x[16] = R0t.m[0][2];
          x[17] = R0t.m[1][2];
          x[18] = R0t.m[2][2];
          x[19] = attPR0t.m[0][2];
          x[20] = attPR0t.m[1][2];
          x[21] = attPR0t.m[2][2];
          x[22] = n1.z;
          x[23] = n2.z;
          x[24] = R0t.m[0][0];
          x[25] = R0t.m[1][0];
          x[26] = R0t.m[2][0];
          x[27] = attP2R0t.m[0][0];
          x[28] = attP2R0t.m[1][0];
          x[29] = attP2R0t.m[2][0];
          x[30] = n3.x;
          x[31] = n4.x;
          x[32] = R0t.m[0][1];
          x[33] = R0t.m[1][1];
          x[34] = R0t.m[2][1];
          x[35] = attP2R0t.m[0][1];
          x[36] = attP2R0t.m[1][1];
          x[37] = attP2R0t.m[2][1];
          x[38] = n3.y;
          x[39] = n4.y;
          x[40] = R0t.m[0][2];
          x[41] = R0t.m[1][2];
          x[42] = R0t.m[2][2];
          x[43] = attP2R0t.m[0][2];
          x[44] = attP2R0t.m[1][2];
          x[45] = attP2R0t.m[2][2];
          x[46] = n3.z;
          x[47] = n4.z;
          x[48] = R0t.m[0][0];
          x[49] = R0t.m[1][0];
          x[50] =  R0t.m[2][0];
          x[51] =  attP3R0t.m[0][0];
          x[52] =  attP3R0t.m[1][0];
          x[53] =  attP3R0t.m[2][0];
          x[54] = n5.x;
          x[55] = n6.x;
          x[56] = R0t.m[0][1];
          x[57] = R0t.m[1][1];
          x[58] =  R0t.m[2][1];
          x[59] =  attP3R0t.m[0][1];
          x[60] =  attP3R0t.m[1][1];
          x[61] =  attP3R0t.m[2][1];
          x[62] = n5.y;
          x[63] = n6.y;
          x[64] = R0t.m[0][2];
          x[65] = R0t.m[1][2];
          x[66] =  R0t.m[2][2];
          x[67] =  attP3R0t.m[0][2];
          x[68] =  attP3R0t.m[1][2];
          x[69] =  attP3R0t.m[2][2];
          x[70] = n5.z;
          x[71] = n6.z;

          osqp_finalize_update(workspace);
          // update q, l, and u (after finalize update!)
          struct vec F_dP = mvmul(R0t, F_d);
          c_float l_new[12] =  {F_dP.x,	F_dP.y,	F_dP.z,  M_d.x,  M_d.y,  M_d.z,  -INFINITY, -INFINITY, -INFINITY, -INFINITY, -INFINITY, -INFINITY,};
          c_float u_new[12] =  {F_dP.x,	F_dP.y,	F_dP.z,  M_d.x,  M_d.y,  M_d.z, 0, 0,  0, 0,  0, 0};

          /* P = np.eye(9)
             1/2 x^2 + lambda (x^2 - 2xx_d + x_d^2)
             => J = (1/2+lambda) x^2 - 2 * lambda x_d x
             =>     1/2 x^2 - 1 * lambda / (1/2+lambda) x_d x
             => q = -2 * lambda / (1+2*lambda) * x_d
          */
          const float factor = - 2.0f * input->self->lambdaa / (1.0f + 2.0f * input->self->lambdaa);
          c_float q_new[9];

          if (input->self->formation_control == 0) {
            q_new[0] = 0.0f;
            q_new[1] = 0.0f;
            q_new[2] = 0.0f;
            q_new[3] = 0.0f;
            q_new[4] = 0.0f;
            q_new[5] = 0.0f;
            q_new[6] = 0.0f;
            q_new[7] = 0.0f;
            q_new[8] = 0.0f;
          }
          else if (input->self->formation_control == 1) {
            q_new[0] = factor * desVirt_prev.x;
            q_new[1] = factor * desVirt_prev.y;
            q_new[2] = factor * desVirt_prev.z;
            q_new[3] = factor * desVirt2_prev.x;
            q_new[4] = factor * desVirt2_prev.y;
            q_new[5] = factor * desVirt2_prev.z;
            q_new[6] = factor * desVirt3_prev.x;
            q_new[7] = factor * desVirt3_prev.y;
            q_new[8] = factor * desVirt3_prev.z;
          } else {
            float scale = vmag(F_d) / 3.0f;
            muDes = vsclnorm(muDes, scale);
            muDes2 = vsclnorm(muDes2, scale);
            muDes3 = vsclnorm(muDes3, scale);

            q_new[0] = factor * muDes.x;
            q_new[1] = factor * muDes.y;
            q_new[2] = factor * muDes.z;
            q_new[3] = factor * muDes2.x;
            q_new[4] = factor * muDes2.y;
            q_new[5] = factor * muDes2.z;
            q_new[6] = factor * muDes3.x;
            q_new[7] = factor * muDes3.y;
            q_new[8] = factor * muDes3.z;
          }

          osqp_update_lin_cost(workspace, q_new);
          osqp_update_lower_bound(workspace, l_new);
          osqp_update_upper_bound(workspace, u_new);

          #ifdef CRAZYFLIE_FW
            uint64_t timestamp_mu_start = usecTimestamp();
          #endif
          osqp_solve(workspace);
          #ifdef CRAZYFLIE_FW
            eventTrigger_qpSolved_payload.mu += (usecTimestamp() - timestamp_mu_start) / 8;
          #endif

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
            output->success = true;
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
          // automatically compute cable length, if desired
          if (l1 <= 0) {
            l1 = vmag(vsub(plStPos, statePos));
          }
          if (l2 <= 0) {
            l2 = vmag(vsub(plStPos, statePos2));
          }
          if (l3 <= 0) {
            l3 = vmag(vsub(plStPos, statePos3));
          }

          // solve QP for 3 uavs 2 hps and point mass
          OSQPWorkspace* workspace = &workspace_3uav_2hp;
          workspace->settings->warm_start = 1;

          struct vec n1, n2, n3, n4, n5, n6;
          if (input->self->gen_hp == 0) {
            n1 = computePlaneNormal(statePos, statePos2, plStPos,  radius, l1, l2); // 1 v 2
            n2 = computePlaneNormal(statePos, statePos3, plStPos,  radius, l1, l3); // 1 v 3
            n3 = computePlaneNormal(statePos2, statePos, plStPos,  radius, l2, l1); // 2 v 1
            n4 = computePlaneNormal(statePos2, statePos3, plStPos, radius, l2, l3); // 2 v 3
            n5 = computePlaneNormal(statePos3, statePos, plStPos,  radius, l3, l1); // 3 v 1
            n6 = computePlaneNormal(statePos3, statePos2, plStPos, radius, l3, l2); // 3 v 2
          } else {
            computePlaneNormals(statePos, statePos2, plStPos, radius, l1, l2, input->self->lambda_svm, F_d, &n1, &n3, &input->self->osqp_warmstart_hyperplanes[0]); // 1 v 2, 2 v 1
            computePlaneNormals(statePos, statePos3, plStPos, radius, l1, l3, input->self->lambda_svm, F_d, &n2, &n5, &input->self->osqp_warmstart_hyperplanes[1]); // 1 v 3, 3 v 1
            computePlaneNormals(statePos2, statePos3, plStPos, radius, l2, l3, input->self->lambda_svm, F_d, &n4, &n6, &input->self->osqp_warmstart_hyperplanes[2]); // 2 v 3, 3 v 2
          }

          osqp_prepare_update(workspace);
          c_float* x = workspace->data->A->x;
          x[1] = n1.x; x[2] = n2.x;
          x[4] = n1.y; x[5] = n2.y;
          x[7] = n1.z; x[8] = n2.z;

          x[10] = n3.x; x[11] = n4.x;
          x[13] = n3.y; x[14] = n4.y;
          x[16] = n3.z; x[17] = n4.z;

          x[19] = n5.x; x[20] = n6.x;
          x[22] = n5.y; x[23] = n6.y;
          x[25] = n5.z; x[26] = n6.z;

          osqp_finalize_update(workspace);

          // update q, l, and u (after finalize update!)
          c_float l_new[9] =  {F_d.x,	F_d.y,	F_d.z, -INFINITY, -INFINITY, -INFINITY, -INFINITY, -INFINITY, -INFINITY,};
          c_float u_new[9] =  {F_d.x,	F_d.y,	F_d.z, 0, 0,  0, 0,  0, 0};

          /* P = np.eye(9)
             1/2 x^2 + lambda (x^2 - 2xx_d + x_d^2)
             => J = (1/2+lambda) x^2 - 2 * lambda x_d x
             =>     1/2 x^2 - 1 * lambda / (1/2+lambda) x_d x
             => q = -2 * lambda / (1+2*lambda) * x_d
          */
          const float factor = - 2.0f * input->self->lambdaa / (1.0f + 2.0f * input->self->lambdaa);
          c_float q_new[9];

          if (input->self->formation_control == 0) {
            q_new[0] = 0.0f;
            q_new[1] = 0.0f;
            q_new[2] = 0.0f;
            q_new[3] = 0.0f;
            q_new[4] = 0.0f;
            q_new[5] = 0.0f;
            q_new[6] = 0.0f;
            q_new[7] = 0.0f;
            q_new[8] = 0.0f;
          }
          else if (input->self->formation_control == 1) {
            q_new[0] = factor * desVirt_prev.x;
            q_new[1] = factor * desVirt_prev.y;
            q_new[2] = factor * desVirt_prev.z;
            q_new[3] = factor * desVirt2_prev.x;
            q_new[4] = factor * desVirt2_prev.y;
            q_new[5] = factor * desVirt2_prev.z;
            q_new[6] = factor * desVirt3_prev.x;
            q_new[7] = factor * desVirt3_prev.y;
            q_new[8] = factor * desVirt3_prev.z;
          } else {
            float scale = vmag(F_d) / 3.0f;
            muDes = vsclnorm(muDes, scale);
            muDes2 = vsclnorm(muDes2, scale);
            muDes3 = vsclnorm(muDes3, scale);

            q_new[0] = factor * muDes.x;
            q_new[1] = factor * muDes.y;
            q_new[2] = factor * muDes.z;
            q_new[3] = factor * muDes2.x;
            q_new[4] = factor * muDes2.y;
            q_new[5] = factor * muDes2.z;
            q_new[6] = factor * muDes3.x;
            q_new[7] = factor * muDes3.y;
            q_new[8] = factor * muDes3.z;
          }

          osqp_update_lin_cost(workspace, q_new);
          osqp_update_lower_bound(workspace, l_new);
          osqp_update_upper_bound(workspace, u_new);

          #ifdef CRAZYFLIE_FW
            uint64_t timestamp_mu_start = usecTimestamp();
          #endif
          osqp_solve(workspace);
          #ifdef CRAZYFLIE_FW
            eventTrigger_qpSolved_payload.mu += (usecTimestamp() - timestamp_mu_start) / 8;
          #endif

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
            output->success = true;
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
#ifdef CRAZYFLIE_FW
  eventTrigger_qpSolved_payload.total = (usecTimestamp() - timestamp_total_start) / 8;
  eventTrigger(&eventTrigger_qpSolved);
#endif
}

static void computeDesiredVirtualInput(controllerLeePayload_t* self, const state_t *state, struct vec F_d, struct vec M_d, uint32_t tick_in, struct vec* result, uint32_t* tick_out)
{
  struct QPInput qpinput;
  struct QPOutput qpoutput;

  // push the latest change to the QP
  qpinput.F_d = F_d;
  qpinput.M_d = M_d;
  qpinput.plStPos = mkvec(state->payload_pos.x, state->payload_pos.y, state->payload_pos.z);
  qpinput.plStquat = mkquat(state->payload_quat.x, state->payload_quat.y, state->payload_quat.z, state->payload_quat.w);
  // if (state->num_neighbors == 1) {
  //   struct vec rpy = quat2rpy(qpinput.plStquat);
  //   rpy.y = 0;
  //   qpinput.plStquat = rpy2quat(rpy);
  // }

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
  qpinput.timestamp = tick_in;

#ifdef CRAZYFLIE_FW
  xQueueOverwrite(queueQPInput, &qpinput);
  // get the latest result from the async computation, do not wait to not block the main loop
  BaseType_t qr = xQueuePeek(queueQPOutput, &qpoutput, 0);

    // static int counter = 0;
    // ++counter;
    // if (counter % 100 == 0) {
    //   DEBUG_PRINT("db %d %d\n", (int)qr, qpoutput.success);
    // }

  // also mark as not successful, if queue was empty for some reason
  qpoutput.success &= (qr == pdTRUE);
#else
  // solve the QP
  runQP(&qpinput, &qpoutput);
#endif
  if (qpoutput.success) {
    *tick_out = qpoutput.timestamp;
    *result = qpoutput.desVirtInp;
  }
}

#ifdef CRAZYFLIE_FW

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
#endif

void controllerLeePayloadReset(controllerLeePayload_t* self)
{
  self->i_error_pos = vzero();
  self->i_error_att = vzero();
  self->i_error_q = vzero();
  self->i_error_pl_att = vzero();
  self->qi_prev = mkvec(0,0,-1);
  self->qidot_prev = vzero();
  self->acc_prev   = vzero();
  self->payload_vel_prev = vzero();
  self->qdi_prev = vzero();
  self->desVirtInp = vzero();
  self->prev_q_des = mkquat(0, 0, 0, 1);
  
  self->delta_bar_x0 = vzero();
  self->delta_bar_R0 = vzero();
  self->delta_bar_xi = vzero();

  #ifdef CRAZYFLIE_FW
  xQueueReset(queueQPInput);
  xQueueReset(queueQPOutput);
  #endif
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

    // struct QPOutput qpoutput;
    // memset(&qpoutput, 0, sizeof(qpoutput));
    // qpoutput.timestamp = 0;
    // qpoutput.desVirtInput = vzero();
    // xQueueOverwrite(queueQPOutput, &qpoutput);

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
    // if (state->num_neighbors == 1) {
    //   struct vec rpy = quat2rpy(plquat);
    //   rpy.y = 0;
    //   plquat = rpy2quat(rpy);
    //   plomega.y = 0;
    // }

    // errors
    struct vec plpos_e = vclampnorm(vsub(plPos_d, plStPos), self->Kpos_P_limit);
    struct vec plvel_e = vclampnorm(vsub(plVel_d, plStVel), self->Kpos_D_limit);
    self->i_error_pos = vclampnorm(vadd(self->i_error_pos, vscl(dt, plpos_e)), self->Kpos_I_limit);

    // Lee's integral error (30)
    self->delta_bar_x0 = vadd(self->delta_bar_x0, vscl(self->h_x0/self->mp*dt, vadd(plvel_e, vscl(self->c_x, plpos_e))));

    struct vec attPoint = mkvec(0, 0, 0);
    float l = -1;
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
        l = self->attachement_points[i].l;
        break;
      }
    }

    // static int counter = 0;
    // if (counter % 100 == 0) {
    //   DEBUG_PRINT("al %f %f %f %f\n", (double)attPoint.x, (double)attPoint.y, (double)attPoint.z, (double)l); 
    // }
    // ++counter;

    if (!isnanf(plquat.w)) {
      // If the payload is a rigid body then the the attachment point should be added to PlStPos
      plStPos = vadd(plStPos, qvrot(plquat, attPoint));
    }

    // automatically compute cable length, if desired
    if (l <= 0) {
      l = vmag(vsub(plStPos, statePos));
    }

    // payload orientation errors
    // payload quat to R 
    struct mat33 Rp = quat2rotmat(plquat);
    // define desired payload Rp_des = eye(3) (i.e., qp_des = [0,0,0,1] (x,y,z,w))

    // Address inconsistency in firmware where we need to compute our own desired yaw angle
    // Rate-controlled YAW is moving YAW angle setpoint
    if (setpoint->mode.yaw == modeVelocity) {
      float desiredYaw = radians(state->attitude.yaw + setpoint->attitudeRate.yaw * dt);
      self->qp_des = rpy2quat(mkvec(0,0,desiredYaw));
    } else if (setpoint->mode.yaw == modeAbs) {
      float desiredYaw = radians(setpoint->attitude.yaw);
      self->qp_des = rpy2quat(mkvec(0,0,desiredYaw));
    } else if (setpoint->mode.quat == modeAbs) {
      self->qp_des = mkquat(setpoint->attitudeQuaternion.x, setpoint->attitudeQuaternion.y, setpoint->attitudeQuaternion.z, setpoint->attitudeQuaternion.w);
    }
  
    struct mat33 Rp_des = quat2rotmat(self->qp_des); 
    // define orientation error     
    // eRp =  msub(mmul(mtranspose(self->R_des), self->R), mmul(mtranspose(self->R), self->R_des));
    struct mat33 eRMp =  msub(mmul(mtranspose(Rp_des), Rp), mmul(mtranspose(Rp), Rp_des));
    struct vec eRp = vscl(0.5f, mkvec(eRMp.m[2][1], eRMp.m[0][2], eRMp.m[1][0]));
    
    self->i_error_pl_att = vclampnorm(vadd(self->i_error_pl_att, vscl(dt, eRp)), self->Kprot_I_limit);

    self->wp_des = mkvec(radians(setpoint->attitudeRate.roll), radians(setpoint->attitudeRate.pitch), radians(setpoint->attitudeRate.yaw));
    self->omega_pr = mvmul(mmul(mtranspose(Rp), Rp_des), self->wp_des);
    struct vec omega_perror = vsub(plomega, self->omega_pr);

    // Lees integral error (31)
    self->delta_bar_R0 = vadd(self->delta_bar_R0, vscl(dt*self->h_R0, vadd(omega_perror, vscl(self->c_R, eRp))));

    self->plp_error = plpos_e;
    self->plv_error = plvel_e;

    // Lee (20)
    // Note that the last component is not included here, since it would require
    // computing delta_bar_xi for all i, on each robot
    self->F_d = vsub(
        vscl(self->mp ,vadd4(
          plAcc_d,
          veltmul(self->Kpos_P, plpos_e),
          veltmul(self->Kpos_D, plvel_e),
          veltmul(self->Kpos_I, self->i_error_pos))), // not in the original formulation
        self->delta_bar_x0
      );

    // Lee (21)
    // Note that the part with omega_0_d and omega_0_d_dot are not included, as they are zero in our case

    // Note that the last component is not included here, since it would require
    // computing delta_bar_xi for all i, on each robot
    self->M_d = vadd4(
      vneg(veltmul(self->Kprot_P, eRp)),
      vneg(veltmul(self->Kprot_D, omega_perror)),
      vneg(veltmul(self->Kprot_I, self->i_error_pl_att)), // not in the original formulation
      vneg(self->delta_bar_R0)
    );

    computeDesiredVirtualInput(self, state, self->F_d, self->M_d, tick, &self->desVirtInp, &self->desVirtInp_tick);

    // static int counter = 0;
    // ++counter;
    // if (counter % 100 == 0) {
    //   DEBUG_PRINT("db %f %f %f\n", (double)self->desVirtInp.x, (double)self->desVirtInp.y, (double)self->desVirtInp.z);
    // }

    // if we don't have a desVirtInp (yet), skip this round
    if (vmag2(self->desVirtInp) == 0) {
      return;
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

    if (self->desVirtInp_tick != self->qdi_prev_tick) {
      if (self->en_qdidot) {
        float qdi_dt = (self->desVirtInp_tick - self->qdi_prev_tick) / 1000.0f;
        self->qdidot = vdiv(vsub(qdi, self->qdi_prev), qdi_dt);
      } else {
        self->qdidot = vzero();
      }
      self->qdi_prev = qdi;
      self->qdi_prev_tick = self->desVirtInp_tick;
    }
    struct vec wdi = vcross(qdi, self->qdidot);
    struct vec ew = vadd(wi, mvmul(skewqi2, wdi));

    // Lee's integral error (32)
    {
      struct vec part1 = vscl(1.0f/self->mp, vadd(plvel_e, vscl(self->c_x, plpos_e)));
      struct vec part2 = vzero();
      if (!isnanf(plquat.w)) {
        struct mat33 part2a = mmul(Rp, mcrossmat(attPoint));
        struct vec part2b = vadd(omega_perror, vscl(self->c_R, eRp));
        part2 = mvmul(part2a, part2b);
      }

      struct vec term1 = vscl(self->h_xi, mvmul(qiqiT, vsub(part1, part2)));

      struct vec term2 = vscl(self->h_xi / self->mass * l, mvmul(skewqi, vadd(ew, vscl(self->c_q, eq))));

      self->delta_bar_xi = vadd(self->delta_bar_xi, vscl(dt, vadd(term1, term2)));
    }

    // Lee (27)
    struct vec u_perpind = vsub2(
      vscl(self->mass*l, mvmul(skewqi, vadd4(
        vneg(veltmul(self->K_q, vclampnorm(eq, self->K_q_limit))),
        vneg(veltmul(self->K_w, vclampnorm(ew, self->K_w_limit))),
        vneg(veltmul(self->K_q_I, self->i_error_q)), // main difference to Lee: Lee multiplies again by skewqui and normalizes by cable length
        vneg(vscl(vdot(self->qi, wdi), self->qidot))))),
      vscl(self->mass, mvmul(skewqi2, acc_)),
      vneg(mvmul(skewqi2, self->delta_bar_xi))
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
    // we only support position control
    control->controlMode = controlModeForceTorque;
    control->thrustSI  = 0;
    control->torque[0] = 0;
    control->torque[1] = 0;
    control->torque[2] = 0;
    controllerLeePayloadReset(self);
    return;
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
  struct vec omega_des;
  if (self->en_num_omega) {
    omega_des = quat2omega(self->prev_q_des, q_des, dt);
    self->prev_q_des = q_des;

  } else {
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
    omega_des = mkvec(-vdot(hw,ydes), vdot(hw,xdes), desiredYawRate);
  }

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
// Attitude Payload I
PARAM_ADD(PARAM_FLOAT, Kprot_Ix, &g_self.Kprot_I.x)
PARAM_ADD(PARAM_FLOAT, Kprot_Iy, &g_self.Kprot_I.y)
PARAM_ADD(PARAM_FLOAT, Kprot_Iz, &g_self.Kprot_I.z)
PARAM_ADD(PARAM_FLOAT, Kprot_I_limit, &g_self.Kprot_I_limit)

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

PARAM_ADD(PARAM_UINT8, en_qdidot, &g_self.en_qdidot)
PARAM_ADD(PARAM_UINT8, en_accrb, &g_self.en_accrb)

PARAM_ADD(PARAM_FLOAT, mass, &g_self.mass)
PARAM_ADD(PARAM_FLOAT, massP, &g_self.mp)

PARAM_ADD(PARAM_FLOAT, radius, &g_self.radius)

// QP tuning

PARAM_ADD(PARAM_FLOAT, lambda, &g_self.lambdaa)
PARAM_ADD(PARAM_FLOAT, lambda_svm, &g_self.lambda_svm)

PARAM_ADD(PARAM_UINT8, gen_hp, &g_self.gen_hp)

PARAM_ADD(PARAM_UINT8, form_ctrl, &g_self.formation_control)

// Attachement points rigid body payload
PARAM_ADD(PARAM_UINT8, ap0id, &g_self.attachement_points[0].id)
PARAM_ADD(PARAM_FLOAT, ap0x, &g_self.attachement_points[0].point.x)
PARAM_ADD(PARAM_FLOAT, ap0y, &g_self.attachement_points[0].point.y)
PARAM_ADD(PARAM_FLOAT, ap0z, &g_self.attachement_points[0].point.z)
PARAM_ADD(PARAM_FLOAT, ap0l, &g_self.attachement_points[0].l)
PARAM_ADD(PARAM_FLOAT, ap0dx, &g_self.attachement_points[0].mu_desired.x)
PARAM_ADD(PARAM_FLOAT, ap0dy, &g_self.attachement_points[0].mu_desired.y)
PARAM_ADD(PARAM_FLOAT, ap0dz, &g_self.attachement_points[0].mu_desired.z)

PARAM_ADD(PARAM_UINT8, ap1id, &g_self.attachement_points[1].id)
PARAM_ADD(PARAM_FLOAT, ap1x, &g_self.attachement_points[1].point.x)
PARAM_ADD(PARAM_FLOAT, ap1y, &g_self.attachement_points[1].point.y)
PARAM_ADD(PARAM_FLOAT, ap1z, &g_self.attachement_points[1].point.z)
PARAM_ADD(PARAM_FLOAT, ap1l, &g_self.attachement_points[1].l)
PARAM_ADD(PARAM_FLOAT, ap1dx, &g_self.attachement_points[1].mu_desired.x)
PARAM_ADD(PARAM_FLOAT, ap1dy, &g_self.attachement_points[1].mu_desired.y)
PARAM_ADD(PARAM_FLOAT, ap1dz, &g_self.attachement_points[1].mu_desired.z)

PARAM_ADD(PARAM_UINT8, ap2id, &g_self.attachement_points[2].id)
PARAM_ADD(PARAM_FLOAT, ap2x, &g_self.attachement_points[2].point.x)
PARAM_ADD(PARAM_FLOAT, ap2y, &g_self.attachement_points[2].point.y)
PARAM_ADD(PARAM_FLOAT, ap2z, &g_self.attachement_points[2].point.z)
PARAM_ADD(PARAM_FLOAT, ap2l, &g_self.attachement_points[2].l)
PARAM_ADD(PARAM_FLOAT, ap2dx, &g_self.attachement_points[2].mu_desired.x)
PARAM_ADD(PARAM_FLOAT, ap2dy, &g_self.attachement_points[2].mu_desired.y)
PARAM_ADD(PARAM_FLOAT, ap2dz, &g_self.attachement_points[2].mu_desired.z)

// Pinv
PARAM_ADD(PARAM_UINT8, Pinv0id1,&g_self.Pinvs[0].id1)
PARAM_ADD(PARAM_UINT8, Pinv0id2,&g_self.Pinvs[0].id2)
PARAM_ADD(PARAM_FLOAT, Pinv000, &g_self.Pinvs[0].Pinv.m[0][0])
PARAM_ADD(PARAM_FLOAT, Pinv010, &g_self.Pinvs[0].Pinv.m[1][0])
PARAM_ADD(PARAM_FLOAT, Pinv020, &g_self.Pinvs[0].Pinv.m[2][0])
PARAM_ADD(PARAM_FLOAT, Pinv030, &g_self.Pinvs[0].Pinv.m[3][0])
PARAM_ADD(PARAM_FLOAT, Pinv040, &g_self.Pinvs[0].Pinv.m[4][0])
PARAM_ADD(PARAM_FLOAT, Pinv050, &g_self.Pinvs[0].Pinv.m[5][0])
PARAM_ADD(PARAM_FLOAT, Pinv001, &g_self.Pinvs[0].Pinv.m[0][1])
PARAM_ADD(PARAM_FLOAT, Pinv011, &g_self.Pinvs[0].Pinv.m[1][1])
PARAM_ADD(PARAM_FLOAT, Pinv021, &g_self.Pinvs[0].Pinv.m[2][1])
PARAM_ADD(PARAM_FLOAT, Pinv031, &g_self.Pinvs[0].Pinv.m[3][1])
PARAM_ADD(PARAM_FLOAT, Pinv041, &g_self.Pinvs[0].Pinv.m[4][1])
PARAM_ADD(PARAM_FLOAT, Pinv051, &g_self.Pinvs[0].Pinv.m[5][1])
PARAM_ADD(PARAM_FLOAT, Pinv002, &g_self.Pinvs[0].Pinv.m[0][2])
PARAM_ADD(PARAM_FLOAT, Pinv012, &g_self.Pinvs[0].Pinv.m[1][2])
PARAM_ADD(PARAM_FLOAT, Pinv022, &g_self.Pinvs[0].Pinv.m[2][2])
PARAM_ADD(PARAM_FLOAT, Pinv032, &g_self.Pinvs[0].Pinv.m[3][2])
PARAM_ADD(PARAM_FLOAT, Pinv042, &g_self.Pinvs[0].Pinv.m[4][2])
PARAM_ADD(PARAM_FLOAT, Pinv052, &g_self.Pinvs[0].Pinv.m[5][2])
PARAM_ADD(PARAM_FLOAT, Pinv003, &g_self.Pinvs[0].Pinv.m[0][3])
PARAM_ADD(PARAM_FLOAT, Pinv013, &g_self.Pinvs[0].Pinv.m[1][3])
PARAM_ADD(PARAM_FLOAT, Pinv023, &g_self.Pinvs[0].Pinv.m[2][3])
PARAM_ADD(PARAM_FLOAT, Pinv033, &g_self.Pinvs[0].Pinv.m[3][3])
PARAM_ADD(PARAM_FLOAT, Pinv043, &g_self.Pinvs[0].Pinv.m[4][3])
PARAM_ADD(PARAM_FLOAT, Pinv053, &g_self.Pinvs[0].Pinv.m[5][3])
PARAM_ADD(PARAM_FLOAT, Pinv004, &g_self.Pinvs[0].Pinv.m[0][4])
PARAM_ADD(PARAM_FLOAT, Pinv014, &g_self.Pinvs[0].Pinv.m[1][4])
PARAM_ADD(PARAM_FLOAT, Pinv024, &g_self.Pinvs[0].Pinv.m[2][4])
PARAM_ADD(PARAM_FLOAT, Pinv034, &g_self.Pinvs[0].Pinv.m[3][4])
PARAM_ADD(PARAM_FLOAT, Pinv044, &g_self.Pinvs[0].Pinv.m[4][4])
PARAM_ADD(PARAM_FLOAT, Pinv054, &g_self.Pinvs[0].Pinv.m[5][4])
PARAM_ADD(PARAM_FLOAT, Pinv005, &g_self.Pinvs[0].Pinv.m[0][5])
PARAM_ADD(PARAM_FLOAT, Pinv015, &g_self.Pinvs[0].Pinv.m[1][5])
PARAM_ADD(PARAM_FLOAT, Pinv025, &g_self.Pinvs[0].Pinv.m[2][5])
PARAM_ADD(PARAM_FLOAT, Pinv035, &g_self.Pinvs[0].Pinv.m[3][5])
PARAM_ADD(PARAM_FLOAT, Pinv045, &g_self.Pinvs[0].Pinv.m[4][5])
PARAM_ADD(PARAM_FLOAT, Pinv055, &g_self.Pinvs[0].Pinv.m[5][5])

PARAM_ADD(PARAM_UINT8, Pinv1id1,&g_self.Pinvs[1].id1)
PARAM_ADD(PARAM_UINT8, Pinv1id2,&g_self.Pinvs[1].id2)
PARAM_ADD(PARAM_FLOAT, Pinv100, &g_self.Pinvs[1].Pinv.m[0][0])
PARAM_ADD(PARAM_FLOAT, Pinv110, &g_self.Pinvs[1].Pinv.m[1][0])
PARAM_ADD(PARAM_FLOAT, Pinv120, &g_self.Pinvs[1].Pinv.m[2][0])
PARAM_ADD(PARAM_FLOAT, Pinv130, &g_self.Pinvs[1].Pinv.m[3][0])
PARAM_ADD(PARAM_FLOAT, Pinv140, &g_self.Pinvs[1].Pinv.m[4][0])
PARAM_ADD(PARAM_FLOAT, Pinv150, &g_self.Pinvs[1].Pinv.m[5][0])
PARAM_ADD(PARAM_FLOAT, Pinv101, &g_self.Pinvs[1].Pinv.m[0][1])
PARAM_ADD(PARAM_FLOAT, Pinv111, &g_self.Pinvs[1].Pinv.m[1][1])
PARAM_ADD(PARAM_FLOAT, Pinv121, &g_self.Pinvs[1].Pinv.m[2][1])
PARAM_ADD(PARAM_FLOAT, Pinv131, &g_self.Pinvs[1].Pinv.m[3][1])
PARAM_ADD(PARAM_FLOAT, Pinv141, &g_self.Pinvs[1].Pinv.m[4][1])
PARAM_ADD(PARAM_FLOAT, Pinv151, &g_self.Pinvs[1].Pinv.m[5][1])
PARAM_ADD(PARAM_FLOAT, Pinv102, &g_self.Pinvs[1].Pinv.m[0][2])
PARAM_ADD(PARAM_FLOAT, Pinv112, &g_self.Pinvs[1].Pinv.m[1][2])
PARAM_ADD(PARAM_FLOAT, Pinv122, &g_self.Pinvs[1].Pinv.m[2][2])
PARAM_ADD(PARAM_FLOAT, Pinv132, &g_self.Pinvs[1].Pinv.m[3][2])
PARAM_ADD(PARAM_FLOAT, Pinv142, &g_self.Pinvs[1].Pinv.m[4][2])
PARAM_ADD(PARAM_FLOAT, Pinv152, &g_self.Pinvs[1].Pinv.m[5][2])
PARAM_ADD(PARAM_FLOAT, Pinv103, &g_self.Pinvs[1].Pinv.m[0][3])
PARAM_ADD(PARAM_FLOAT, Pinv113, &g_self.Pinvs[1].Pinv.m[1][3])
PARAM_ADD(PARAM_FLOAT, Pinv123, &g_self.Pinvs[1].Pinv.m[2][3])
PARAM_ADD(PARAM_FLOAT, Pinv133, &g_self.Pinvs[1].Pinv.m[3][3])
PARAM_ADD(PARAM_FLOAT, Pinv143, &g_self.Pinvs[1].Pinv.m[4][3])
PARAM_ADD(PARAM_FLOAT, Pinv153, &g_self.Pinvs[1].Pinv.m[5][3])
PARAM_ADD(PARAM_FLOAT, Pinv104, &g_self.Pinvs[1].Pinv.m[0][4])
PARAM_ADD(PARAM_FLOAT, Pinv114, &g_self.Pinvs[1].Pinv.m[1][4])
PARAM_ADD(PARAM_FLOAT, Pinv124, &g_self.Pinvs[1].Pinv.m[2][4])
PARAM_ADD(PARAM_FLOAT, Pinv134, &g_self.Pinvs[1].Pinv.m[3][4])
PARAM_ADD(PARAM_FLOAT, Pinv144, &g_self.Pinvs[1].Pinv.m[4][4])
PARAM_ADD(PARAM_FLOAT, Pinv154, &g_self.Pinvs[1].Pinv.m[5][4])
PARAM_ADD(PARAM_FLOAT, Pinv105, &g_self.Pinvs[1].Pinv.m[0][5])
PARAM_ADD(PARAM_FLOAT, Pinv115, &g_self.Pinvs[1].Pinv.m[1][5])
PARAM_ADD(PARAM_FLOAT, Pinv125, &g_self.Pinvs[1].Pinv.m[2][5])
PARAM_ADD(PARAM_FLOAT, Pinv135, &g_self.Pinvs[1].Pinv.m[3][5])
PARAM_ADD(PARAM_FLOAT, Pinv145, &g_self.Pinvs[1].Pinv.m[4][5])
PARAM_ADD(PARAM_FLOAT, Pinv155, &g_self.Pinvs[1].Pinv.m[5][5])

PARAM_ADD(PARAM_UINT8, Pinv2id1,&g_self.Pinvs[2].id1)
PARAM_ADD(PARAM_UINT8, Pinv2id2,&g_self.Pinvs[2].id2)
PARAM_ADD(PARAM_FLOAT, Pinv200, &g_self.Pinvs[2].Pinv.m[0][0])
PARAM_ADD(PARAM_FLOAT, Pinv210, &g_self.Pinvs[2].Pinv.m[1][0])
PARAM_ADD(PARAM_FLOAT, Pinv220, &g_self.Pinvs[2].Pinv.m[2][0])
PARAM_ADD(PARAM_FLOAT, Pinv230, &g_self.Pinvs[2].Pinv.m[3][0])
PARAM_ADD(PARAM_FLOAT, Pinv240, &g_self.Pinvs[2].Pinv.m[4][0])
PARAM_ADD(PARAM_FLOAT, Pinv250, &g_self.Pinvs[2].Pinv.m[5][0])
PARAM_ADD(PARAM_FLOAT, Pinv201, &g_self.Pinvs[2].Pinv.m[0][1])
PARAM_ADD(PARAM_FLOAT, Pinv211, &g_self.Pinvs[2].Pinv.m[1][1])
PARAM_ADD(PARAM_FLOAT, Pinv221, &g_self.Pinvs[2].Pinv.m[2][1])
PARAM_ADD(PARAM_FLOAT, Pinv231, &g_self.Pinvs[2].Pinv.m[3][1])
PARAM_ADD(PARAM_FLOAT, Pinv241, &g_self.Pinvs[2].Pinv.m[4][1])
PARAM_ADD(PARAM_FLOAT, Pinv251, &g_self.Pinvs[2].Pinv.m[5][1])
PARAM_ADD(PARAM_FLOAT, Pinv202, &g_self.Pinvs[2].Pinv.m[0][2])
PARAM_ADD(PARAM_FLOAT, Pinv212, &g_self.Pinvs[2].Pinv.m[1][2])
PARAM_ADD(PARAM_FLOAT, Pinv222, &g_self.Pinvs[2].Pinv.m[2][2])
PARAM_ADD(PARAM_FLOAT, Pinv232, &g_self.Pinvs[2].Pinv.m[3][2])
PARAM_ADD(PARAM_FLOAT, Pinv242, &g_self.Pinvs[2].Pinv.m[4][2])
PARAM_ADD(PARAM_FLOAT, Pinv252, &g_self.Pinvs[2].Pinv.m[5][2])
PARAM_ADD(PARAM_FLOAT, Pinv203, &g_self.Pinvs[2].Pinv.m[0][3])
PARAM_ADD(PARAM_FLOAT, Pinv213, &g_self.Pinvs[2].Pinv.m[1][3])
PARAM_ADD(PARAM_FLOAT, Pinv223, &g_self.Pinvs[2].Pinv.m[2][3])
PARAM_ADD(PARAM_FLOAT, Pinv233, &g_self.Pinvs[2].Pinv.m[3][3])
PARAM_ADD(PARAM_FLOAT, Pinv243, &g_self.Pinvs[2].Pinv.m[4][3])
PARAM_ADD(PARAM_FLOAT, Pinv253, &g_self.Pinvs[2].Pinv.m[5][3])
PARAM_ADD(PARAM_FLOAT, Pinv204, &g_self.Pinvs[2].Pinv.m[0][4])
PARAM_ADD(PARAM_FLOAT, Pinv214, &g_self.Pinvs[2].Pinv.m[1][4])
PARAM_ADD(PARAM_FLOAT, Pinv224, &g_self.Pinvs[2].Pinv.m[2][4])
PARAM_ADD(PARAM_FLOAT, Pinv234, &g_self.Pinvs[2].Pinv.m[3][4])
PARAM_ADD(PARAM_FLOAT, Pinv244, &g_self.Pinvs[2].Pinv.m[4][4])
PARAM_ADD(PARAM_FLOAT, Pinv254, &g_self.Pinvs[2].Pinv.m[5][4])
PARAM_ADD(PARAM_FLOAT, Pinv205, &g_self.Pinvs[2].Pinv.m[0][5])
PARAM_ADD(PARAM_FLOAT, Pinv215, &g_self.Pinvs[2].Pinv.m[1][5])
PARAM_ADD(PARAM_FLOAT, Pinv225, &g_self.Pinvs[2].Pinv.m[2][5])
PARAM_ADD(PARAM_FLOAT, Pinv235, &g_self.Pinvs[2].Pinv.m[3][5])
PARAM_ADD(PARAM_FLOAT, Pinv245, &g_self.Pinvs[2].Pinv.m[4][5])
PARAM_ADD(PARAM_FLOAT, Pinv255, &g_self.Pinvs[2].Pinv.m[5][5])


// Lee's integral gains
PARAM_ADD(PARAM_FLOAT, h_x0, &g_self.h_x0)
PARAM_ADD(PARAM_FLOAT, h_R0, &g_self.h_R0)
PARAM_ADD(PARAM_FLOAT, h_xi, &g_self.h_xi)
PARAM_ADD(PARAM_FLOAT, c_x, &g_self.c_x)
PARAM_ADD(PARAM_FLOAT, c_R, &g_self.c_R)
PARAM_ADD(PARAM_FLOAT, c_q, &g_self.c_q)

// more fun flags
PARAM_ADD(PARAM_UINT8, en_num_w, &g_self.en_num_omega)
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

// computed virtual input
LOG_ADD(LOG_FLOAT, desVirtInp2x, &g_self.desVirtInp2.x)
LOG_ADD(LOG_FLOAT, desVirtInp2y, &g_self.desVirtInp2.y)
LOG_ADD(LOG_FLOAT, desVirtInp2z, &g_self.desVirtInp2.z)

LOG_ADD(LOG_UINT32, profQP, &qp_runtime_us)

LOG_GROUP_STOP(ctrlLeeP)

#endif // CRAZYFLIE_FW defined


               


          
