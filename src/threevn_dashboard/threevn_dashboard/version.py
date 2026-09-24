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
What this robot is running.

Debugging a physical system without this is guesswork: the behaviour in
front of you cannot be tied to a commit, a configuration or a deployment
unless the robot can say which ones it has.

Values come from the environment, injected at build or deploy time. They
are deliberately NOT read from the local git checkout at runtime - a
deployed robot has no checkout, and a value that silently falls back to
whatever happens to be on disk is worse than one that honestly says
'unknown'.
"""

from datetime import datetime, timezone
import os
import socket

#: Bumped by hand at release. Distinct from the git commit, which changes
#: on every push.
SOFTWARE_VERSION = '0.16.0'

UNKNOWN = 'unknown'


def _env(name, default=UNKNOWN):
    """Read an env var, treating empty strings as absent."""
    value = os.environ.get(name, '').strip()
    return value or default


def version_info():
    """
    Return the full version block.

    Every field a failure report needs in order to be actionable.
    """
    return {
        'robot_id': _env('THREEVN_ROBOT_ID', socket.gethostname()),
        'software_version': SOFTWARE_VERSION,
        # Set by CI at image build time. 'unknown' means this is a
        # developer build, which is itself useful to know.
        'git_commit': _env('THREEVN_GIT_COMMIT'),
        'git_branch': _env('THREEVN_GIT_BRANCH'),
        # No firmware exists until Phase 5. Reporting 'unknown' rather
        # than a plausible-looking zero keeps the gap visible.
        'firmware_version': _env('THREEVN_FIRMWARE_VERSION'),
        'robot_model': _env('THREEVN_ROBOT_MODEL', 'threevn_arm_v1'),
        'config_profile': _env('THREEVN_PROFILE', 'threevn_arm_v1'),
        'hardware_target': _env('THREEVN_TARGET'),
        'deployed_at': _env('THREEVN_DEPLOYED_AT'),
        'reported_at': datetime.now(timezone.utc).isoformat(timespec='seconds'),
    }


def missing_fields():
    """
    Return the version fields that are still 'unknown'.

    Surfaced on the dashboard rather than hidden: a robot that cannot say
    which commit it is running is a robot you cannot debug, and that
    should be visible before something goes wrong rather than after.
    """
    info = version_info()
    return sorted(k for k, v in info.items() if v == UNKNOWN)
