#!/usr/bin/env python

import numpy as np
import cffirmware

def test_that_vec_is_converted_to_numpy_array():
    # Fixture
    v_cf = cffirmware.mkvec(1, 2, 3)

    # Test
    actual = np.array(v_cf)

    # Assert
    expected = np.array([1, 2, 3])
    assert np.allclose(expected, actual)

# def test_bla():
#     # Fixture
#     q0 = cffirmware.mkquat(0,0,0,1)
#     omega = cffirmware.mkvec(10,20,30)
#     dt = 0.01

#     # Test
#     actual = cffirmware.quat_gyro_update(q0, omega, dt)
#     # omega1 = cffirmware.quat2omega(q0, q1, dt)

#     # Assert
#     expected = cffirmware.mkquat(-0.14912653, 0.09941769, -0.04970884, 0.98255098)

#     assert np.allclose(np.array(actual), np.array(expected))

def test_quat2omega():
    # Fixture
    q0 = cffirmware.mkquat(0,0,0,1)
    q1 = cffirmware.mkquat(0.04970884, 0.09941769, 0.14912653, 0.98255098)
    # omega = cffirmware.mkvec(10,20,30)
    dt = 0.01

    # Test
    omega = cffirmware.quat2omega(q0, q1, dt)
    actual = np.array(omega)

    # Assert
    expected = np.array([10,20,30])

    assert np.allclose(actual, expected, atol=0.2)
