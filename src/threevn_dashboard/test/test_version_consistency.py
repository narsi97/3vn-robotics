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
Every declared software version agrees with VERSION.

This exists because of a real drift, not a hypothetical one. A running
robot reported `software_version: 0.3.0` while all six package manifests
said `0.1.0`. Nothing broke, which is exactly the problem: the version
block exists to be trusted, and nobody would have noticed it was lying
until they tried to reproduce a fault from it.

The firmware and protocol versions are deliberately NOT checked against
this. They are separate artifacts with separate lifecycles - a host
updated a dozen times can still be talking to firmware nobody reflashed,
and forcing one number would hide precisely that. See
docs/versioning.md.
"""

import pathlib
import re
import xml.etree.ElementTree as ET

import pytest
from threevn_dashboard.version import SOFTWARE_VERSION


def _repo_root():
    """Walk up until VERSION is found, so this works from any cwd."""
    here = pathlib.Path(__file__).resolve()
    for parent in here.parents:
        if (parent / 'VERSION').is_file():
            return parent
    pytest.skip('VERSION not found; running outside a source checkout')


@pytest.fixture(scope='module')
def root():
    """Provide the repository root."""
    return _repo_root()


@pytest.fixture(scope='module')
def declared(root):
    """Provide the single declared software version."""
    return (root / 'VERSION').read_text().strip()


def test_version_file_is_semver(declared):
    """VERSION holds a plain semantic version, nothing else."""
    assert re.fullmatch(r'\d+\.\d+\.\d+', declared), \
        f'VERSION contains {declared!r}, which is not a bare semver'


def test_every_package_manifest_matches(root, declared):
    """
    All six package.xml files declare the same version.

    A release built from mismatched manifests produces packages that
    disagree about what they are, which surfaces later as a rosdep or
    packaging error far from the cause.
    """
    manifests = sorted(root.glob('src/*/package.xml'))
    assert manifests, 'no package manifests found'
    wrong = {}
    for manifest in manifests:
        version = ET.parse(manifest).getroot().findtext('version')
        if version != declared:
            wrong[manifest.relative_to(root).as_posix()] = version
    assert not wrong, (
        f'these declare a version other than {declared}: {wrong}. '
        'VERSION is the only place it should be written.'
    )


def test_every_setup_py_matches(root, declared):
    """The ament_python packages agree too."""
    wrong = {}
    for setup in sorted(root.glob('src/*/setup.py')):
        match = re.search(r"version\s*=\s*'([^']+)'", setup.read_text())
        if match and match.group(1) != declared:
            wrong[setup.relative_to(root).as_posix()] = match.group(1)
    assert not wrong, f'these disagree with {declared}: {wrong}'


def test_the_robot_reports_the_declared_version(declared):
    """
    What the robot says it is running matches what was built.

    This is the assertion that would have caught the original drift: the
    dashboard reported 0.3.0 from a tree that declared 0.1.0 everywhere
    else, and only the dashboard's number is visible in the field.
    """
    assert SOFTWARE_VERSION == declared, (
        f'the dashboard reports {SOFTWARE_VERSION} but VERSION says '
        f'{declared}; a robot would be telling you the wrong thing'
    )


def test_firmware_and_protocol_versions_are_separate_on_purpose(root):
    """
    Firmware and protocol versions exist and are NOT tied to VERSION.

    Asserted so nobody "helpfully" unifies them later. A host updated
    many times can still face firmware nobody reflashed; one number
    would hide that, which is the situation the version block is meant
    to expose.
    """
    controller = (root / 'firmware/esp32/include/controller.hpp').read_text()
    assert re.search(r'kFirmwareMajor\s*=\s*\d+', controller)
    assert re.search(r'kFirmwareMinor\s*=\s*\d+', controller)

    protocol = (
        root / 'src/threevn_hardware/include/threevn_hardware/protocol.hpp'
    ).read_text()
    assert re.search(r'kProtocolVersion\s*=\s*\d+', protocol)
