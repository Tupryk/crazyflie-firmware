%module cffirmware

%include "stdint.i"
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

// ignore GNU specific compiler attributes
#define __attribute__(x)

%{
#define SWIG_FILE_WITH_INIT
#include "math3d.h"
#include "imu_types.h"
#include "stabilizer_types.h"
#include "kalman_core.h"
#include "mm_position.h"
%}

%include "math3d.h"
%include "imu_types.h"
%include "stabilizer_types.h"
%include "kalman_core.h"
%include "mm_position.h"

%inline %{
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
