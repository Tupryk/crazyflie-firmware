%module cffirmware
%include <stdint.i>

%include "cpointer.i"
%include "carrays.i"
// %typemap(in) float[ANY] (float temp[$1_dim0]) {
//   int i;
//   if (!PySequence_Check($input)) {
//     PyErr_SetString(PyExc_ValueError, "Expected a sequence");
//     SWIG_fail;
//   }
//   if (PySequence_Length($input) != $1_dim0) {
//     PyErr_SetString(PyExc_ValueError, "Size mismatch. Expected $1_dim0 elements");
//     SWIG_fail;
//   }
//   for (i = 0; i < $1_dim0; i++) {
//     PyObject *o = PySequence_GetItem($input, i);
//     if (PyNumber_Check(o)) {
//       temp[i] = (float) PyFloat_AsDouble(o);
//     } else {
//       PyErr_SetString(PyExc_ValueError, "Sequence elements must be numbers");      
//       SWIG_fail;
//     }
//   }
//   $1 = temp;
// }

%typemap(memberin) float [ANY] {
  int i;
  for (i = 0; i < $1_dim0; i++) {
      $1[i] = $input[i];
  }
}

%typemap(out) float [ANY] {
  int i;
  $result = PyList_New($1_dim0);
  for (i = 0; i < $1_dim0; i++) {
    PyObject *o = PyFloat_FromDouble((double) $1[i]);
    PyList_SetItem($result,i,o);
  }
}

// ignore GNU specific compiler attributes
#define __attribute__(x)

%{
#define SWIG_FILE_WITH_INIT
#include "math3d.h"
#include "pptraj.h"
#include "planner.h"
#include "stabilizer_types.h"
#include "collision_avoidance.h"
#include "imu_types.h"
#include "controller_pid.h"
#include "position_controller.h"
#include "pid.h"
#include "filter.h"
#include "num.h"
#include "controller_mellinger.h"
#include "power_distribution.h"
#include "controller_sjc.h"
#include "controller_lee.h"
#include "controller_lee_payload.h"
%}

%include "math3d.h"
%include "pptraj.h"
%include "planner.h"
%include "stabilizer_types.h"
%include "collision_avoidance.h"
%include "controller_pid.h"
%include "imu_types.h"
%include "controller_mellinger.h"
%include "power_distribution.h"
%include "controller_sjc.h"
%include "controller_lee.h"
%include "controller_lee_payload.h"

%inline %{
struct poly4d* piecewise_get(struct piecewise_traj *pp, int i)
{
    return &pp->pieces[i];
}
void poly4d_set(struct poly4d *poly, int dim, int coef, float val)
{
    poly->p[dim][coef] = val;
}
float poly4d_get(struct poly4d *poly, int dim, int coef)
{
    return poly->p[dim][coef];
}
struct poly4d* poly4d_malloc(int size)
{
    return (struct poly4d*)malloc(sizeof(struct poly4d) * size);
}
void poly4d_free(struct poly4d *p)
{
    free(p);
}

struct vec vec2svec(struct vec3_s v)
{
    return mkvec(v.x, v.y, v.z);
}

struct vec3_s svec2vec(struct vec v)
{
    struct vec3_s vv = {
        .x = v.x,
        .y = v.y,
        .z = v.z,
    };
    return vv;
}

void collisionAvoidanceUpdateSetpointWrap(
    collision_avoidance_params_t const *params,
    collision_avoidance_state_t *collisionState,
    int nOthers,
    float const *otherPositions,
    setpoint_t *setpoint, sensorData_t const *sensorData, state_t const *state)
{
    nOthers /= 3;
    float *workspace = malloc(sizeof(float) * 7 * (nOthers + 6));
    collisionAvoidanceUpdateSetpointCore(
        params,
        collisionState,
        nOthers,
        otherPositions,
        workspace,
        setpoint, sensorData, state);
    free(workspace);
}

void state_set_position(state_t *state, int idx, uint8_t id, float x, float y, float z)
{
    state->team_state[idx].id = id;
    state->team_state[idx].pos.x = x;
    state->team_state[idx].pos.y = y;
    state->team_state[idx].pos.z = z;
}

struct vec controller_lee_payload_get_n(controllerLeePayload_t* self, int idx) 
{
    struct vec n = mkvec(self->n[idx].x, self->n[idx].y, self->n[idx].z);
    return n;
}
void set_setpoint_qi_ref(setpoint_t *setpoint, int idx, uint8_t id, float qx, float qy, float qz, float qdx, float qdy, float qdz, float wdx, float wdy, float wdz) 
{     
    setpoint->cablevectors[idx].id = id;
    setpoint->cablevectors[idx].mu_planned.x = qx;
    setpoint->cablevectors[idx].mu_planned.y = qy;
    setpoint->cablevectors[idx].mu_planned.z = qz;
    setpoint->cablevectors[idx].qid_ref.x = qdx;
    setpoint->cablevectors[idx].qid_ref.y = qdy;
    setpoint->cablevectors[idx].qid_ref.z = qdz;

    setpoint->cablevectors[idx].wid_ref.x = wdx;
    setpoint->cablevectors[idx].wid_ref.y = wdy;
    setpoint->cablevectors[idx].wid_ref.z = wdz;
}

void set_rot_des(controllerLeePayload_t* self, float x00, float x01, float x02, float x10, float x11, float x12, float x20, float x21, float x22)
{
    self->R_des.m[0][0] = x00;
    self->R_des.m[0][1] = x01;
    self->R_des.m[0][2] = x02;
    self->R_des.m[1][0] = x10;
    self->R_des.m[1][1] = x11;
    self->R_des.m[1][2] = x12;
    self->R_des.m[2][0] = x20;
    self->R_des.m[2][1] = x21;
    self->R_des.m[2][2] = x22;

}


void controller_lee_payload_set_attachement(controllerLeePayload_t* self, int idx, uint8_t id, float x, float y, float z)
{
    self->attachement_points[idx].id = id;
    self->attachement_points[idx].point.x = x;
    self->attachement_points[idx].point.y = y;
    self->attachement_points[idx].point.z = z;
}

void controller_lee_payload_set_Pinv(controllerLeePayload_t* self, int idx, int id1, int id2, int row, int column, float value)
{
    self->Pinvs[idx].id1 = id1;
    self->Pinvs[idx].id2 = id2;
    self->Pinvs[idx].Pinv.m[row][column] = value;
}
%}

%pythoncode %{
import numpy as np
%}

#define COPY_CTOR(structname) \
structname(struct structname const *x) { \
    struct structname *y = malloc(sizeof(struct structname)); \
    *y = *x; \
    return y; \
} \
~structname() { \
    free($self); \
} \

%extend vec {
    COPY_CTOR(vec)

    %pythoncode %{
        def __repr__(self):
            return "({}, {}, {})".format(self.x, self.y, self.z)

        def __array__(self):
            return np.array([self.x, self.y, self.z])

        def __len__(self):
            return 3

        def __getitem__(self, i):
            if 0 <= i and i < 3:
                return _cffirmware.vindex(self, i)
            else:
                raise IndexError("vec index must be in {0, 1, 2}.")

        # Unary operator overloads.
        def __neg__(self):
            return _cffirmware.vneg(self)

        # Vector-scalar binary operator overloads.
        def __rmul__(self, s):
            return _cffirmware.vscl(s, self)

        def __div__(self, s):
            return self.__truediv__(s)

        def __truediv__(self, s):
            return _cffirmware.vdiv(self, s)

        # Vector-vector binary operator overloads.
        def __add__(self, other):
            return _cffirmware.vadd(self, other)

        def __sub__(self, other):
            return _cffirmware.vsub(self, other)
    %}
};

%extend traj_eval {
    COPY_CTOR(traj_eval)
};
