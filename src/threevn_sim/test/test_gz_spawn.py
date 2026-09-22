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
Gazebo integration: the robot spawns and reports joint state.

Marked `slow` and excluded from `make test`; `make test-sim` runs it.
Starting a simulator takes tens of seconds, and a test suite that takes
minutes is a test suite nobody runs before committing.
"""
import pytest

pytestmark = pytest.mark.slow


def test_placeholder_until_phase_2():
    """
    Phase 1 ships the world, the launch file and the description that.

    targets Gazebo, but the spawn-and-verify test belongs to Phase 2,
    where the simulation itself is the deliverable.

    This is a deliberate marker, not an oversight: writing an assertion
    here now would either duplicate Phase 2's work or assert something
    trivially true. See docs/roadmap.md.
    """
    pytest.skip('Gazebo spawn verification lands in Phase 2')
