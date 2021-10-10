"""Compiles the cffirmware C extension."""

from distutils.core import setup, Extension
import os

fw_dir = "."
include = [
    os.path.join(fw_dir, "src/modules/interface"),
    os.path.join(fw_dir, "src/modules/interface/kalman_core"),
    os.path.join(fw_dir, "src/utils/interface"),
    os.path.join(fw_dir, "src/utils/interface/lighthouse"),
    os.path.join(fw_dir, "src/hal/interface"),
    os.path.join(fw_dir, "bindings"),
]

fw_sources = [
    # list firmware c-files here
    "src/modules/src/kalman_core/kalman_core.c",
    "src/modules/src/kalman_core/mm_position.c",
    "bindings/arm_math.c",
]

cffirmware = Extension(
    "_cffirmware",
    include_dirs=include,
    sources=fw_sources + ["bin/cffirmware_wrap.c"],
    extra_compile_args=[
        "-O0",
        "-g",
        "-DUNIT_TEST_MODE"
    ],
)

setup(name="cffirmware", version="1.0", ext_modules=[cffirmware])
