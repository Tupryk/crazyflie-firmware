"""Compiles the cffirmware C extension."""

import distutils.command.build
from distutils.core import setup, Extension
import os

fw_dir = "."
include = [
    os.path.join(fw_dir, "src/modules/interface"),
    os.path.join(fw_dir, "src/hal/interface"),
    os.path.join(fw_dir, "src/utils/interface/lighthouse"),
    os.path.join(fw_dir, "src/modules/interface/kalman_core"),
    os.path.join(fw_dir, "src/utils/interface"),
    os.path.join(fw_dir, "src/utils/interface/lighthouse"),
    os.path.join(fw_dir, "src/hal/interface"),
    os.path.join(fw_dir, "bindings"),
]

fw_sources = [
    "src/modules/src/pptraj.c",
    "src/modules/src/pptraj_compressed.c",
    "src/modules/src/planner.c",
    "src/modules/src/collision_avoidance.c",
    "src/modules/src/kalman_core/kalman_core.c",
    "src/modules/src/kalman_core/mm_position.c",
    "bindings/arm_math.c",
]
# fw_sources = [os.path.join(fw_dir, "src/modules/src", mod) for mod in modules]

cffirmware = Extension(
    "_cffirmware",
    include_dirs=include,
    sources=fw_sources + ["bin/cffirmware_wrap.c"],
    extra_compile_args=[
        "-O3",
        # The following flags are also used for compiling the actual firmware
        "-fno-strict-aliasing",
        "-Wno-address-of-packed-member",
        "-DUNIT_TEST_MODE",
    ],
)

# Override build command to specify custom "build" directory
class BuildCommand(distutils.command.build.build):
    def initialize_options(self):
        distutils.command.build.build.initialize_options(self)
        self.build_base = "bin"

setup(
    name="cffirmware",
    version="1.0",
    cmdclass={"build": BuildCommand},
    ext_modules=[cffirmware]
)
