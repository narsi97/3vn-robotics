# Versioning

Three version numbers, deliberately independent, all reported together.

| Number | Source of truth | Changes when |
|---|---|---|
| **software** | [`VERSION`](../VERSION) | any release of the ROS packages |
| **firmware** | `firmware/esp32/include/controller.hpp` | the ESP32 image changes |
| **protocol** | `src/threevn_hardware/include/threevn_hardware/protocol.hpp` | the wire format changes incompatibly |

They are separate because they are separate artifacts with separate
lifecycles. A host that has been updated twelve times can still be
talking to firmware nobody has reflashed, and forcing one number would
hide exactly that.

What matters is that all three are **reported together**, by the running
robot at `/api/version` and in every release manifest. Debugging a
physical system means tying observed behaviour to a specific combination,
and a version you cannot trust is worse than no version at all.

## One software version, asserted

`VERSION` is the only place the software version is written. Every
`package.xml`, every `setup.py`, and `threevn_dashboard/version.py` must
agree with it, and `test_version_consistency.py` fails the build if they
do not.

This was not theoretical. Before it existed, a robot reported
`software_version: 0.3.0` while all six package manifests said `0.1.0`.
Nothing broke, which is the problem — the version block is there to be
trusted, and nobody would have noticed it was lying until they tried to
reproduce a fault from it.

## The scheme, while it is pre-1.0

The minor version tracks the implementation phase: `0.7.0` is Phase 7.
That holds until the platform runs on a physical arm, at which point
semver starts meaning what it usually means and the phase mapping is
retired.

**1.0.0 requires a robot that moves.** Not a passing simulation.

## Protocol version

`kProtocolVersion` is bumped only when a frame's layout changes in a way
an older peer would misread. The firmware **refuses** a frame whose
version it does not implement rather than guessing at the layout —
guessing means moving servos on misread numbers.

Because `protocol.hpp` is compiled into both the host and the firmware,
the two cannot disagree about what version 1 means. They can disagree
about *which* version they speak, which is precisely what the field is
for.

## Releases

A tag `vX.Y.Z` triggers [`release.yml`](../.github/workflows/release.yml),
which refuses to run if the tag and `VERSION` disagree. The release
carries a manifest recording all three numbers plus the image digests, so
a deployed robot can always be traced back to an exact set of artifacts.
