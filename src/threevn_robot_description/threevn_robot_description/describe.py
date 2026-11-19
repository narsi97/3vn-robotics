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
Turn a robot profile into the URDF the stack runs.

    ros2 run threevn_robot_description describe so101_follower --target gz

ONE function, `describe()`, is how every launch file and every
integration test gets a robot description. It exists because there are
now two ways a description comes to be, and a caller should not have to
know which applies:

  * A robot we designed (`meta.kind` arm, base, mobile_manipulator) is
    GENERATED from its profile by xacro. The YAML is the source of
    physical truth.

  * A robot somebody else designed (`meta.kind: vendor_arm`) arrives as
    a finished URDF. The profile is an OVERLAY: it adds the things this
    stack needs and a plain URDF has no place for, and changes nothing
    about the kinematics.

What the overlay adds, and nothing more:

  * a `world` anchor, because a free-floating arm topples
  * absolute mesh paths, because `assets/x.stl` resolves against the
    caller's working directory rather than the file's
  * effort and velocity limits derived from the servo datasheet, in
    place of upstream's placeholders
  * the `<ros2_control>` block for the chosen target -- the hardware
    seam, exactly as arm.ros2_control.xacro draws it for our own arm
  * a `<threevn_semantics>` block: which joints are the arm, how wide
    the gripper opens, and the named poses

The upstream file itself is never edited. It is fetched unmodified into
vendor/ and read from there.
"""

import argparse
import math
import os
import pathlib
import struct
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import xacro
import yaml

PKG = 'threevn_robot_description'

#: xacro entry point per robot family we generate ourselves.
ENTRY_POINTS = {
    'arm': 'threevn_arm.urdf.xacro',
    'base': 'threevn_base.urdf.xacro',
    'mobile_manipulator': 'threevn_mobile_manipulator.urdf.xacro',
}

#: Families described by somebody else's URDF plus an overlay.
VENDOR_KINDS = {'vendor_arm'}

#: Hardware plugin per target for a vendor arm. `feetech` is read from
#: the profile instead, so that choosing a driver is configuration.
VENDOR_PLUGINS = {
    'mock': 'mock_components/GenericSystem',
    'gz': 'gz_ros2_control/GazeboSimSystem',
}

#: The controllers file a profile gets when it names none.
DEFAULT_CONTROLLERS = 'controllers.yaml'


class DescriptionUnavailable(RuntimeError):
    """Raised when a profile cannot be turned into a description."""


def share():
    """Return this package's share directory."""
    return pathlib.Path(get_package_share_directory(PKG))


def vendor_root():
    """
    Return the directory third-party descriptions are fetched into.

    /ws/vendor in the dev container, where compose bind-mounts the
    repository's gitignored vendor/ directory.
    """
    return pathlib.Path(os.environ.get('THREEVN_VENDOR_DIR', '/ws/vendor'))


def profile_path(profile):
    """
    Resolve a profile name or path to the YAML file.

    Accepts `so101_follower`, `so101_follower.yaml`, or a path, so a
    Make variable and a test fixture can both be passed straight in.
    """
    candidate = pathlib.Path(str(profile))
    if candidate.suffix == '.yaml' and candidate.exists():
        return candidate
    name = candidate.name
    if not name.endswith('.yaml'):
        name += '.yaml'
    path = share() / 'config' / name
    if not path.exists():
        known = sorted(p.stem for p in (share() / 'config').glob('*.yaml'))
        raise DescriptionUnavailable(
            f'no robot profile {profile!r}; one of {known}')
    return path


def load_profile(profile):
    """Return a profile as a dict."""
    return yaml.safe_load(profile_path(profile).read_text())


def controllers_file(profile):
    """
    Return the NAME of the controllers file a profile runs under.

    A name, not a path: the file lives in threevn_bringup, which depends
    on this package, so this package must not look it up. The launch
    files join it onto bringup's share directory.
    """
    cfg = load_profile(profile)
    return (cfg.get('control') or {}).get('controllers', DEFAULT_CONTROLLERS)


def model_name(profile):
    """Return the name a profile's robot is spawned and reported under."""
    cfg = load_profile(profile)
    if cfg['meta']['kind'] in VENDOR_KINDS:
        return cfg['meta']['model']
    # Our own arm has always been spawned as `threevn_arm`, whichever
    # profile built it, and the tests and docs say so.
    return 'threevn_arm'


def targets(profile):
    """Return the execution targets a profile can be described for."""
    cfg = load_profile(profile)
    if cfg['meta']['kind'] in VENDOR_KINDS:
        return ['mock', 'gz'] + sorted(cfg.get('hardware') or {})
    return ['mock', 'gz', 'esp32']


def hardware_target(profile):
    """
    Return the name of the target that is this robot's real hardware.

    `esp32` for our own arm, and for a vendor arm whatever its profile
    declares under `hardware:` -- exactly one, or this refuses to guess.
    """
    cfg = load_profile(profile)
    if cfg['meta']['kind'] not in VENDOR_KINDS:
        return 'esp32'
    declared = sorted(cfg.get('hardware') or {})
    if len(declared) != 1:
        raise DescriptionUnavailable(
            f'{cfg["meta"]["model"]} declares {len(declared)} hardware '
            f'targets ({declared}); name one')
    return declared[0]


def hardware_parameters(profile, target=None, overrides=None):
    """
    Return a vendor arm's hardware parameters, with overrides applied.

    The profile says where the robot's bus normally is. `overrides`
    replaces individual parameters for one run -- the endpoint, most
    often -- without editing the profile: the simulated bus is on a TCP
    port, a Mac reaches the real one through another, and a Linux host
    opens the serial device the profile names.

    An override for a parameter the profile does not declare is refused.
    A typo would otherwise be passed to the plugin and silently ignored
    there, and the robot would start on the default it was meant to
    avoid.
    """
    cfg = load_profile(profile)
    target = target or hardware_target(profile)
    declared = (cfg.get('hardware') or {}).get(target)
    if declared is None:
        raise DescriptionUnavailable(
            f'{cfg["meta"]["model"]} has no hardware target {target!r}')
    merged = dict(declared)
    for key, value in (overrides or {}).items():
        if value in (None, ''):
            continue
        if key not in declared or key == 'plugin':
            raise DescriptionUnavailable(
                f'{cfg["meta"]["model"]}: {key!r} is not a parameter of '
                f'the {target} target; one of '
                f'{sorted(k for k in declared if k != "plugin")}')
        merged[key] = value
    return merged


def describe(profile, target='mock', controllers='', prefix='',
             hardware=None):
    """
    Return the URDF for `profile` on `target`, as a string.

    `controllers` is the path of the ros2_control parameters file. Only
    the gz target uses it, because only there is controller_manager
    hosted by the description (inside the simulator) rather than started
    beside it by the launch file.

    `hardware` overrides individual hardware parameters of a vendor
    arm's real target for this one description. See
    hardware_parameters().
    """
    path = profile_path(profile)
    cfg = yaml.safe_load(path.read_text())
    kind = cfg['meta']['kind']
    overrides = {k: v for k, v in (hardware or {}).items()
                 if v not in (None, '')}
    if kind in VENDOR_KINDS:
        if prefix:
            raise DescriptionUnavailable(
                f'{cfg["meta"]["model"]}: a vendor description cannot be '
                f'prefixed; its frame names belong to upstream')
        if overrides and target in VENDOR_PLUGINS:
            raise DescriptionUnavailable(
                f'{cfg["meta"]["model"]}: hardware parameters '
                f'{sorted(overrides)} were given for target {target!r}, '
                f'which has no hardware')
        if target not in VENDOR_PLUGINS:
            # Validates the target and the overrides before any file is
            # read, and hands the merged result on.
            cfg = dict(cfg)
            cfg['hardware'] = dict(cfg.get('hardware') or {})
            if target in cfg['hardware']:
                cfg['hardware'][target] = hardware_parameters(
                    path, target, overrides)
        return _describe_vendor(cfg, target, controllers)
    if overrides:
        raise DescriptionUnavailable(
            f'{path.stem}: hardware parameters cannot be overridden for a '
            f'robot described by xacro; they are read from its profile')
    if kind not in ENTRY_POINTS:
        raise DescriptionUnavailable(
            f'{path.name}: unknown meta.kind {kind!r}')
    doc = xacro.process_file(
        str(share() / 'urdf' / ENTRY_POINTS[kind]),
        mappings={'params_file': str(path), 'target': target,
                  'prefix': prefix, 'controllers_file': controllers})
    # Exactly what `ros2 run xacro xacro` prints, so a description built
    # here and one built on the command line are the same bytes.
    return doc.toprettyxml(indent='  ')


# -- vendor arms -------------------------------------------------------

def _describe_vendor(cfg, target, controllers):
    model = cfg['meta']['model']
    plugin = _vendor_plugin(cfg, target)

    source = vendor_root() / cfg['description']['urdf']
    if not source.exists():
        raise DescriptionUnavailable(
            f'{source} is missing. Run '
            f'`{cfg["description"].get("fetch", "the fetch script")}` '
            f'first: the {cfg["meta"]["display_name"]} description is '
            f'{cfg["description"]["licence"]} upstream and deliberately '
            f'not committed to this repository.')

    root = ET.fromstring(source.read_text())
    root.set('name', model)

    _simplify_collisions(root, cfg['description'].get('collision'),
                         source.parent)
    _absolute_mesh_paths(root, source.parent)
    _derive_limits(root, cfg)
    _add_joint_dynamics(root, cfg)
    _anchor(root, cfg['description'].get('anchor'))
    root.append(_ros2_control(root, cfg, target, plugin))
    if target == 'gz':
        root.append(_gz_plugin(controllers))
    root.append(_semantics(cfg))

    ET.indent(root, space='  ')
    return ('<?xml version="1.0" ?>\n'
            + ET.tostring(root, encoding='unicode') + '\n')


def _vendor_plugin(cfg, target):
    if target in VENDOR_PLUGINS:
        return VENDOR_PLUGINS[target]
    hardware = cfg.get('hardware') or {}
    if target in hardware:
        return hardware[target]['plugin']
    raise DescriptionUnavailable(
        f'{cfg["meta"]["model"]} has no target {target!r}; one of '
        f'{sorted(VENDOR_PLUGINS) + sorted(hardware)}')


def _absolute_mesh_paths(root, base):
    """
    Point every relative mesh at the file it means.

    Upstream writes `assets/x.stl`, relative to the URDF. That works for
    a tool handed the file and fails for one handed the STRING, which is
    how both the simulator's spawner and robot_state_publisher receive
    it.
    """
    for mesh in root.iter('mesh'):
        name = mesh.get('filename', '')
        if '://' not in name and not name.startswith('/'):
            mesh.set('filename', f'file://{(base / name).resolve()}')


def stl_bounds(path):
    """
    Return the axis-aligned bounds of an STL mesh: (min xyz, max xyz).

    Binary or ASCII. Read with the standard library alone, because this
    runs at launch on the robot image too, and a mesh library is a lot
    of dependency for six numbers.
    """
    data = pathlib.Path(path).read_bytes()
    points = []
    count = struct.unpack_from('<I', data, 80)[0] if len(data) >= 84 else 0
    if count and len(data) == 84 + 50 * count:
        # Binary: 80-byte header, a count, then per triangle a normal,
        # three vertices and a two-byte attribute.
        for record in struct.iter_unpack('<12fH', data[84:]):
            points.extend((record[3:6], record[6:9], record[9:12]))
    else:
        for line in data.decode('ascii', errors='ignore').splitlines():
            parts = line.split()
            if len(parts) == 4 and parts[0] == 'vertex':
                points.append(tuple(float(v) for v in parts[1:]))
    if not points:
        raise DescriptionUnavailable(f'{path}: no triangles in this STL')
    lows = tuple(min(p[i] for p in points) for i in range(3))
    highs = tuple(max(p[i] for p in points) for i in range(3))
    return lows, highs


def _rotate(rpy, vector):
    """Rotate `vector` by URDF roll-pitch-yaw: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    roll, pitch, yaw = rpy
    x, y, z = vector
    y, z = (y * math.cos(roll) - z * math.sin(roll),
            y * math.sin(roll) + z * math.cos(roll))
    x, z = (x * math.cos(pitch) + z * math.sin(pitch),
            -x * math.sin(pitch) + z * math.cos(pitch))
    x, y = (x * math.cos(yaw) - y * math.sin(yaw),
            x * math.sin(yaw) + y * math.cos(yaw))
    return x, y, z


def _simplify_collisions(root, policy, base):
    """
    Replace mesh collisions with something a CPU can afford.

    Upstream's collision geometry is its visual geometry: the full
    printed-part meshes, tens of thousands of triangles a link. That is
    right for a renderer and ruinous for contact physics on a CPU --
    measured here, the arm as shipped runs at about an eighth of real
    time, where our own primitive-built arm manages most of it.

    `policy` chooses per link: `mesh` keeps upstream's, `box` swaps each
    mesh for its own bounding box in the same place, `none` removes it.
    A box is conservative -- it is never smaller than the part -- so an
    arm that clears the bench as boxes clears it as plastic.

    Visuals are never touched, and neither are the files in vendor/.
    """
    if not policy:
        return
    default = policy.get('default', 'mesh')
    per_link = policy.get('links') or {}
    for link in root.iter('link'):
        mode = per_link.get(link.get('name'), default)
        if mode == 'mesh':
            continue
        for collision in list(link.findall('collision')):
            mesh = collision.find('geometry/mesh')
            if mode == 'none':
                link.remove(collision)
            elif mode == 'box' and mesh is not None:
                _box_for(collision, mesh, base)
            elif mode not in ('none', 'box'):
                raise DescriptionUnavailable(
                    f'unknown collision mode {mode!r} for {link.get("name")}; '
                    f'one of mesh, box, none')


def _box_for(collision, mesh, base):
    """Turn one mesh collision into its bounding box, in place."""
    scale = [float(v) for v in (mesh.get('scale') or '1 1 1').split()]
    lows, highs = stl_bounds(base / mesh.get('filename'))
    lows = [lo * s for lo, s in zip(lows, scale)]
    highs = [hi * s for hi, s in zip(highs, scale)]
    lows, highs = ([min(a, b) for a, b in zip(lows, highs)],
                   [max(a, b) for a, b in zip(lows, highs)])
    size = [hi - lo for lo, hi in zip(lows, highs)]
    centre = [(lo + hi) / 2.0 for lo, hi in zip(lows, highs)]

    # The box is centred on the mesh's bounds, which are not at the mesh
    # origin, so the collision origin moves by that centre -- expressed
    # in the link frame, hence the rotation.
    origin = collision.find('origin')
    if origin is None:
        origin = ET.SubElement(collision, 'origin', xyz='0 0 0', rpy='0 0 0')
    xyz = [float(v) for v in (origin.get('xyz') or '0 0 0').split()]
    rpy = [float(v) for v in (origin.get('rpy') or '0 0 0').split()]
    shift = _rotate(rpy, centre)
    origin.set('xyz', ' '.join(f'{a + b:.6g}' for a, b in zip(xyz, shift)))

    geometry = collision.find('geometry')
    geometry.remove(mesh)
    ET.SubElement(geometry, 'box', size=' '.join(f'{v:.6g}' for v in size))


def joint_limits_for(cfg, joint):
    """
    Return (effort, velocity) for a joint, derived from its servo.

    The servo's stall torque and no-load speed, scaled by the profile's
    derating. A joint the profile does not list keeps upstream's values
    and returns None.
    """
    spec = (cfg.get('joints') or {}).get(joint)
    if not spec:
        return None
    servo = cfg['servos'][spec['servo']]
    derate = cfg['limit_derate']
    return (round(servo['stall_torque_nm'] * derate['effort'], 4),
            round(servo['no_load_speed_rads'] * derate['velocity'], 4))


def _derive_limits(root, cfg):
    for joint in root.findall('joint'):
        limit = joint.find('limit')
        derived = joint_limits_for(cfg, joint.get('name'))
        if limit is None or derived is None:
            continue
        limit.set('effort', str(derived[0]))
        limit.set('velocity', str(derived[1]))


def _add_joint_dynamics(root, cfg):
    """
    Give every servo-driven joint the friction a gearbox has.

    Upstream declares no <dynamics>, so its joints are frictionless
    hinges. A servo behind a 1:345 gearbox is nothing like one, and the
    difference is not cosmetic in simulation -- see `defaults` in the
    SO-101 profile for what a frictionless jaw did.

    A joint that already declares <dynamics> upstream keeps it.
    """
    defaults = cfg.get('defaults') or {}
    damping = defaults.get('joint_damping')
    friction = defaults.get('joint_friction')
    if damping is None and friction is None:
        return
    # findall, not iter: upstream's <transmission> blocks contain <joint>
    # elements of their own, and those are not joints.
    for joint in root.findall('joint'):
        if joint.get('name') not in (cfg.get('joints') or {}):
            continue
        if joint.find('dynamics') is not None:
            continue
        ET.SubElement(joint, 'dynamics', damping=str(damping or 0.0),
                      friction=str(friction or 0.0))


def _anchor(root, anchor):
    if not anchor:
        return
    link = ET.Element('link', name=anchor['parent'])
    joint = ET.Element('joint', name=f'{anchor["parent"]}_to_{anchor["child"]}',
                       type='fixed')
    ET.SubElement(joint, 'parent', link=anchor['parent'])
    ET.SubElement(joint, 'child', link=anchor['child'])
    ET.SubElement(joint, 'origin',
                  xyz=' '.join(str(v) for v in anchor['xyz']),
                  rpy=' '.join(str(v) for v in anchor['rpy']))
    # First, so the tree reads root-down.
    root.insert(0, joint)
    root.insert(0, link)


def controlled_joints(cfg):
    """Return every commanded joint: the arm's, then the gripper's."""
    return list(cfg['arm']['joints']) + [cfg['gripper']['joint']]


def _ros2_control(root, cfg, target, plugin):
    """
    Build the hardware seam for a vendor arm.

    The same shape arm.ros2_control.xacro gives our own arm: one plugin
    chosen by `target`, and per-joint interfaces whose limits mirror the
    <limit> tags, here read from upstream's URDF.
    """
    limits = {}
    for joint in root.findall('joint'):
        limit = joint.find('limit')
        if limit is not None:
            limits[joint.get('name')] = (limit.get('lower'), limit.get('upper'))

    block = ET.Element('ros2_control', name=cfg['meta']['model'], type='system')
    hardware = ET.SubElement(block, 'hardware')
    ET.SubElement(hardware, 'plugin').text = plugin
    if target == 'mock':
        _param(hardware, 'calculate_dynamics', 'true')
    elif target not in VENDOR_PLUGINS:
        for key, value in cfg['hardware'][target].items():
            if key != 'plugin':
                _param(hardware, key, value)

    for name in controlled_joints(cfg):
        if name not in limits:
            raise DescriptionUnavailable(
                f'{cfg["meta"]["model"]}: the profile names joint {name!r}, '
                f'which the upstream URDF does not have or does not limit '
                f'(it has {sorted(limits)})')
        joint = ET.SubElement(block, 'joint', name=name)
        reports = _self_reported(cfg, name)
        if target not in VENDOR_PLUGINS:
            # Only real hardware needs to know which servo on the bus,
            # and how that servo's counts map onto this joint's radians.
            spec = cfg['joints'][name]
            _param(joint, 'id', spec['id'])
            for key in ('zero_count', 'direction'):
                if key in spec:
                    _param(joint, key, spec[key])
            if 'load' in reports:
                # The servo reports load as a share of what it can
                # deliver. 100 % is the joint's effort limit, derived
                # from the same datasheet and derating, so a load of
                # 0.9 and an effort of 0.9 x limit are the same claim.
                _param(joint, 'full_load_effort', joint_limits_for(cfg, name)[0])
        command = ET.SubElement(joint, 'command_interface', name='position')
        _param(command, 'min', limits[name][0])
        _param(command, 'max', limits[name][1])
        state = ET.SubElement(joint, 'state_interface', name='position')
        _param(state, 'initial_value', '0.0')
        ET.SubElement(joint, 'state_interface', name='velocity')
        # Effort, where the servo measures its own. Every target gets it,
        # so the dashboard's strain check can be rehearsed in Gazebo,
        # where the physics engine reports the effort a joint is
        # applying, before it is relied on with a real arm. Mock has no
        # physics and reports zero.
        if 'load' in reports:
            ET.SubElement(joint, 'state_interface', name='effort')
        # Temperature only from real servos: nothing simulates one, and a
        # simulated 25 C would be a number that looks measured.
        if 'temperature' in reports and target not in VENDOR_PLUGINS:
            ET.SubElement(joint, 'state_interface', name='temperature')
    return block


def _self_reported(cfg, joint):
    """Return what a joint's servo measures about itself, from its datasheet."""
    spec = (cfg.get('joints') or {}).get(joint) or {}
    servo = (cfg.get('servos') or {}).get(spec.get('servo')) or {}
    return set(servo.get('feedback') or ())


def _param(parent, name, value):
    ET.SubElement(parent, 'param', name=name).text = str(value)


def _gz_plugin(controllers):
    """Build the gz_ros2_control system plugin, as gz_control_plugin.xacro does."""
    gazebo = ET.Element('gazebo')
    plugin = ET.SubElement(
        gazebo, 'plugin', filename='gz_ros2_control-system',
        name='gz_ros2_control::GazeboSimROS2ControlPlugin')
    if controllers:
        ET.SubElement(plugin, 'parameters').text = str(controllers)
    ET.SubElement(plugin, 'controller_manager_name').text = 'controller_manager'
    return gazebo


def _semantics(cfg):
    """
    Build the <threevn_semantics> block.

    The same element urdf/arm/semantics.xacro emits for our own arm; the
    client reads either without knowing which produced it.
    """
    arm, gripper = cfg['arm'], cfg['gripper']
    block = ET.Element('threevn_semantics', profile=cfg['meta']['model'])
    ET.SubElement(block, 'arm', joints=' '.join(arm['joints']),
                  base_frame=arm['base_frame'], tool_frame=arm['tool_frame'])
    ET.SubElement(block, 'gripper', joint=gripper['joint'],
                  open=str(gripper['open']), closed=str(gripper['closed']))
    for name, positions in cfg['poses'].items():
        if len(positions) != len(arm['joints']):
            raise DescriptionUnavailable(
                f'{cfg["meta"]["model"]}: pose {name!r} has {len(positions)} '
                f'positions for {len(arm["joints"])} arm joints')
        ET.SubElement(block, 'pose', name=name,
                      positions=' '.join(str(float(p)) for p in positions))
    return block


def parse_semantics(urdf_xml):
    """
    Read the <threevn_semantics> block back out of a description.

    Returns a dict with `profile`, `arm_joints`, `base_frame`,
    `tool_frame`, `gripper` and `poses`, or None when the description
    carries no such block.
    """
    root = ET.fromstring(urdf_xml)
    block = root.find('threevn_semantics')
    if block is None:
        return None
    arm, gripper = block.find('arm'), block.find('gripper')
    return {
        'profile': block.get('profile'),
        'arm_joints': arm.get('joints').split(),
        'base_frame': arm.get('base_frame'),
        'tool_frame': arm.get('tool_frame'),
        'gripper': {'joint': gripper.get('joint'),
                    'open': float(gripper.get('open')),
                    'closed': float(gripper.get('closed'))},
        'poses': {pose.get('name'):
                  [float(v) for v in pose.get('positions').split()]
                  for pose in block.findall('pose')},
    }


def robot_facts(profile, target='mock', controllers=''):
    """
    Return what an integration test needs to know about a robot.

    Read out of the DESCRIPTION, not out of the profile: the tests
    drive a running robot, and what that robot is running is the
    description. A test that took joint names from the YAML would be
    comparing the profile with itself.
    """
    urdf = describe(profile, target=target, controllers=controllers)
    root = ET.fromstring(urdf)
    semantics = parse_semantics(urdf)
    if semantics is None:
        raise DescriptionUnavailable(
            f'{profile}: its description carries no <threevn_semantics>, '
            f'so the tests cannot tell which joints are the arm')
    limits, mimics = {}, {}
    for joint in root.iter('joint'):
        limit = joint.find('limit')
        if limit is not None and limit.get('lower') is not None:
            limits[joint.get('name')] = (float(limit.get('lower')),
                                         float(limit.get('upper')))
        mimic = joint.find('mimic')
        if mimic is not None:
            mimics[joint.get('name')] = mimic.get('joint')
    gripper = semantics['gripper']['joint']
    return {
        'profile': str(profile),
        'kind': load_profile(profile)['meta']['kind'],
        'urdf': urdf,
        'model': model_name(profile),
        'hardware': [block.get('name') for block in root.findall('ros2_control')],
        'arm_joints': semantics['arm_joints'],
        'gripper_joint': gripper,
        'gripper': semantics['gripper'],
        'gripper_followers': sorted(
            follower for follower, leader in mimics.items() if leader == gripper),
        'limits': limits,
        'base_frame': semantics['base_frame'],
        'tool_frame': semantics['tool_frame'],
        'poses': semantics['poses'],
    }


def preflight(profile, overrides=None, exists=os.path.exists):
    """
    Return None if the robot's hardware can be reached, else why not.

    Asked BEFORE the control stack is launched. Without it, a serial
    device that is not there surfaces as controller_manager aborting
    with an uncaught runtime_error, which reads as a crash rather than
    as "nothing is plugged in".

    Only a serial endpoint can be checked from here: it is a path, and
    it exists or it does not. A TCP endpoint is checked by connecting,
    and the plugin does that with a timeout and its own message.
    """
    cfg = load_profile(profile)
    target = hardware_target(profile)
    params = hardware_parameters(profile, target, overrides)
    if params.get('transport') != 'serial' or exists(params['endpoint']):
        return None
    name = cfg['meta']['display_name']
    return (
        f'\n  No device at {params["endpoint"]} inside the container.\n\n'
        f'  Plug in the servo adapter of the {name}, then pass it through '
        f'to Docker.\n'
        f'  On macOS, Docker Desktop cannot forward USB directly: serve the '
        f'adapter\n'
        f'  from the host as a TCP port and use  TRANSPORT=tcp '
        f'ENDPOINT=host.docker.internal:<port>\n'
        f'  Override the path with:              ENDPOINT=/dev/ttyUSB0\n'
        f'  Rehearse with nothing plugged in:    make fake-bus   then   '
        f'make robot PROFILE={pathlib.Path(str(profile)).stem} BUS=fake\n')


def main(argv=None):
    """Print a profile's URDF and nothing else, so it can be piped."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('profile', help='profile name or path')
    parser.add_argument('--target', default='mock')
    parser.add_argument('--controllers', default='',
                        help='ros2_control parameters file (gz target)')
    parser.add_argument('--prefix', default='')
    parser.add_argument('--transport', default='',
                        help='override the hardware transport for this run')
    parser.add_argument('--endpoint', default='',
                        help='override the hardware endpoint for this run')
    parser.add_argument('--hardware-target', action='store_true',
                        help='print the name of the real-hardware target')
    parser.add_argument('--preflight', action='store_true',
                        help='check the hardware is reachable; print nothing '
                             'and exit 0 if it is')
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])
    overrides = {'transport': args.transport, 'endpoint': args.endpoint}
    try:
        if args.hardware_target:
            print(hardware_target(args.profile))
            return 0
        if args.preflight:
            problem = preflight(args.profile, overrides)
            if problem:
                print(problem, file=sys.stderr)
                return 1
            return 0
        sys.stdout.write(describe(args.profile, target=args.target,
                                  controllers=args.controllers,
                                  prefix=args.prefix, hardware=overrides))
    except DescriptionUnavailable as exc:
        print(f'describe: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
