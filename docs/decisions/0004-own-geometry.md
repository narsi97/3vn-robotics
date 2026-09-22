# ADR-0004: Our own primitive geometry, not third-party meshes

**Status:** accepted · **Date:** 2026-09-22

## Context

The original specification named the EEZYbotARM family as the reference
design. Checking its licence first — as the spec itself instructs —
showed both the original and the MK2 are **CC BY-NC 4.0**, verified on
the live Thingiverse pages. NonCommercial is incompatible with the
planned paid course.

## Decision

3VN Arm v1 uses **our own geometry**: boxes and cylinders, with every
dimension, mass and limit in `config/*.yaml`.

Five independent reasons, any one sufficient:

1. **Licence.** No third-party CAD means no NonCommercial term, and no
   argument about whether a derived mesh or a measured URDF is a
   derivative work. The repository is cleanly Apache-2.0.
2. **Collision cost.** Boxes and cylinders are analytic in DART. A mesh
   becomes convex decomposition or a BVH. On a CPU-only llvmpipe
   container that is the difference between real-time and 0.3×.
3. **CI.** No binary blobs, no Git LFS, no mesh-load failures in a
   headless container. The repository stays diffable and around 1 MB.
4. **Pedagogy.** The course can teach "inertia follows from the shape"
   because the shape *is* the primitive — every number is checkable by
   hand, and `test_inertia_valid.py` checks them automatically.
5. **Honesty.** No physical arm exists yet. A mesh would imply precision
   we do not have. Primitives plus YAML say "this is what we currently
   believe", and updating it after fabrication is a one-line diff.

Every mass carries a `provenance:` field (`measured`, `datasheet`,
`estimated`), required by `test_config_schema.py`. Today they are all
`estimated`, and that is visible in the data rather than hidden in a
comment.

## Escape hatch

`meshes/` exists and is empty. When a physical arm exists and we own its
CAD, drop STLs in and add a `use_meshes` xacro argument. Nothing in the
current design blocks it.

## For students who own an EEZYbotARM

Printing one for personal learning is non-commercial use and entirely
within its licence. Measure your printed arm and write the numbers into
a profile YAML — the parameter mechanism exists precisely so the software
is not welded to one set of dimensions. We link to the design; we ship no
files from it, and no profile derived from its published dimensions.
