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


def test_normalize_radians():
    # Fixture
    angles = [-100, -5, 0, np.pi + 0.1, -np.pi - 0.1, 100]

    for angle in angles:
        # Test
        actual = cffirmware.normalize_radians(angle)
        # Assert
        expected = np.arctan2(np.sin(angle), np.cos(angle))
        assert np.allclose(expected, actual)


def test_shortest_signed_angle_radians():
    # Fixture
    angle_pairs = [(-np.pi/2, np.pi), (np.pi/2, np.pi), (np.pi, -np.pi/3)]

    for start, goal in angle_pairs:
        # Test
        actual = cffirmware.shortest_signed_angle_radians(start, goal)
        # Assert
        expected = np.arctan2(np.sin(goal - start), np.cos(goal - start))
        assert np.allclose(expected, actual)
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
