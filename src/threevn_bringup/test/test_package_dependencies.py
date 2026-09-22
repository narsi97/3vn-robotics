# Copyright 2026 3VN Systems
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
The architectural boundary, enforced by CI rather than by discipline.

threevn_bringup is what runs on the real robot. If it ever gains a Gazebo
dependency, `rosdep install` on the robot would pull in the entire
simulator, and the project's central non-negotiable -- that the
application is not coupled to its execution target -- would be quietly
false.

A comment cannot prevent that. This test can.
"""
import pathlib
import re
import xml.etree.ElementTree as ET


PKG_XML = pathlib.Path(__file__).resolve().parents[1] / 'package.xml'

#: Anything matching these is simulator-specific.
FORBIDDEN = re.compile(r'(^|_)(gz|gazebo|ignition)(_|$)|ros_gz|gz_ros2_control')

DEPEND_TAGS = (
    'depend', 'build_depend', 'buildtool_depend', 'exec_depend',
    'build_export_depend', 'run_depend',
)


def _declared_dependencies():
    root = ET.parse(PKG_XML).getroot()
    for tag in DEPEND_TAGS:
        for el in root.findall(tag):
            if el.text:
                yield tag, el.text.strip()


def test_bringup_never_depends_on_gazebo():
    offenders = [
        f'{tag}: {name}'
        for tag, name in _declared_dependencies()
        if FORBIDDEN.search(name)
    ]
    assert not offenders, (
        'threevn_bringup declares a Gazebo dependency: '
        f'{offenders}. Simulator-specific code belongs in threevn_sim, '
        'which depends on this package -- not the reverse.'
    )


def test_bringup_depends_on_the_description():
    names = {name for _, name in _declared_dependencies()}
    assert 'threevn_robot_description' in names


def test_test_dependencies_are_not_forbidden_by_accident():
    """
    Sanity-check the matcher itself.

    It must not reject ordinary names, or the test above would pass for
    the wrong reason.
    """
    for benign in ('controller_manager', 'robot_state_publisher', 'xacro'):
        assert not FORBIDDEN.search(benign), f'matcher wrongly flags {benign}'
    for bad in ('ros_gz', 'gz_ros2_control', 'ros_gz_sim', 'gazebo_ros'):
        assert FORBIDDEN.search(bad), f'matcher misses {bad}'
