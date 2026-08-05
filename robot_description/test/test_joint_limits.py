"""Sanity check: assert every movable joint's type and limits match the
design spec below.

The values here are hand-typed independently from urdf/*.xacro, not read
back out of it -- comparing the xacro to itself could never catch a typo
introduced there. This exists to catch exactly that: a design/# swap, a
copy-paste that forgot to update one limit, a sign error, etc, before it
turns into a confusing downstream bug in Phase 3-5.
"""

import math
import os

import pytest
import xacro
from ament_index_python.packages import get_package_share_directory
from urdf_parser_py.urdf import URDF

# name -> (type, lower, upper). lower/upper are None for continuous/fixed
# joints, which have no limit tag to check.
EXPECTED_JOINTS = {
    'base_footprint_joint': ('fixed', None, None),
    'left_wheel_joint': ('continuous', None, None),
    'right_wheel_joint': ('continuous', None, None),
    'caster_joint': ('fixed', None, None),
    'base_yaw_joint': ('revolute', -2.9671, 2.9671),
    'shoulder_pitch_joint': ('revolute', -1.5708, 1.5708),
    'elbow_pitch_joint': ('revolute', 0.0, 2.5),
    'wrist_roll_joint': ('continuous', None, None),
    'wrist_pitch_joint': ('revolute', -1.745, 1.745),
    'wrist_roll_2_joint': ('continuous', None, None),
    'left_finger_joint': ('prismatic', 0.0, 0.04),
    'right_finger_joint': ('prismatic', -0.04, 0.0),
}

TOLERANCE = 1e-6


@pytest.fixture(scope='module')
def robot_model():
    pkg_share = get_package_share_directory('robot_description')
    xacro_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    doc = xacro.process_file(xacro_path)
    return URDF.from_xml_string(doc.toxml())


def test_no_missing_or_unexpected_joints(robot_model):
    actual_names = {j.name for j in robot_model.joints}
    expected_names = set(EXPECTED_JOINTS.keys())
    missing = expected_names - actual_names
    unexpected = actual_names - expected_names
    assert not missing, f'Joints in spec but not in URDF: {missing}'
    assert not unexpected, f'Joints in URDF but not in spec: {unexpected}'


@pytest.mark.parametrize('joint_name', EXPECTED_JOINTS.keys())
def test_joint_matches_spec(robot_model, joint_name):
    expected_type, expected_lower, expected_upper = EXPECTED_JOINTS[joint_name]
    joint = next((j for j in robot_model.joints if j.name == joint_name), None)
    assert joint is not None, f'{joint_name} not found in parsed URDF'

    assert joint.type == expected_type, (
        f'{joint_name}: expected type "{expected_type}", got "{joint.type}"'
    )

    if expected_lower is None and expected_upper is None:
        return

    assert joint.limit is not None, f'{joint_name}: expected limits, but <limit> tag is missing'
    assert math.isclose(joint.limit.lower, expected_lower, abs_tol=TOLERANCE), (
        f'{joint_name}: expected lower={expected_lower}, got {joint.limit.lower}'
    )
    assert math.isclose(joint.limit.upper, expected_upper, abs_tol=TOLERANCE), (
        f'{joint_name}: expected upper={expected_upper}, got {joint.limit.upper}'
    )


def test_gripper_mimic_relationship(robot_model):
    right_finger = next(j for j in robot_model.joints if j.name == 'right_finger_joint')
    assert right_finger.mimic is not None, 'right_finger_joint should mimic left_finger_joint'
    assert right_finger.mimic.joint == 'left_finger_joint'
    assert math.isclose(right_finger.mimic.multiplier, -1.0, abs_tol=TOLERANCE)
