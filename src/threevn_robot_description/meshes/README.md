# Meshes

**Empty on purpose**, and this file exists so the directory survives a
clone — git does not track empty directories, and `CMakeLists.txt`
installs this path.

That is not a hypothetical: the first CI run on a fresh clone failed with
`ament_cmake_symlink_install_directory() can't find .../meshes`. It had
always worked locally because the directory was created by hand.

3VN Arm v1 uses primitive boxes and cylinders rather than meshes. That is
a deliberate choice — licence cleanliness, analytic collision geometry,
and no binary blobs in git. See
[ADR-0004](../../../docs/decisions/0004-own-geometry.md).

When a physical arm exists and we own its CAD, drop STLs here and add the
`use_meshes` xacro argument described in that ADR.
