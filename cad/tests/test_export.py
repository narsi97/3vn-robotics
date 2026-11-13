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
Whether the output directory tells the truth about the design.

The exporter only ever added files. When `shoulder_bracket` became
linkage-only -- direct drive carries that servo in the turret -- its
STL stayed in `out/direct/` and every later `make cad` left it there,
with a timestamp no more suspicious than its neighbours'. PRINTING.md
is generated from the part list and correctly omitted it, so the
manifest said eleven parts and the directory offered twelve.

That gap is the dangerous kind: it is invisible unless you compare two
things nobody thinks to compare, and the thing a person actually drags
into a slicer is the directory. 21 g of plastic for a part with no job,
carrying a vertical servo pocket no joint on this arm wants.

These tests exercise the sweep directly rather than through `generate`,
which would build every solid in the mechanism for a question that is
entirely about filenames.
"""

import pytest
from threevn_cad import export
from threevn_cad import parts as parts_mod


@pytest.fixture
def out_dir(tmp_path):
    """An output directory as a previous run would have left it."""
    for name in ('base_plate', 'shoulder_turret', 'shoulder_bracket'):
        (tmp_path / f'{name}.stl').write_text('solid\n')
        (tmp_path / f'{name}.step').write_text('ISO-10303-21;\n')
    (tmp_path / export.ASSEMBLY).write_text('ISO-10303-21;\n')
    (tmp_path / 'PRINTING.md').write_text('# Printing\n')
    return tmp_path


def keep_for(mechanism):
    """The filenames a run of `mechanism` is entitled to leave behind."""
    keep = {export.ASSEMBLY}
    for name in parts_mod.parts_for(mechanism):
        keep.update((f'{name}.stl', f'{name}.step'))
    return keep


def test_removes_a_part_this_mechanism_no_longer_has(out_dir):
    """The bug, stated as a test: direct drive has no shoulder_bracket."""
    assert 'shoulder_bracket' not in parts_mod.parts_for('direct')

    removed = export.prune_stale(out_dir, keep_for('direct'))

    assert sorted(removed) == ['shoulder_bracket.step',
                               'shoulder_bracket.stl']
    assert not (out_dir / 'shoulder_bracket.stl').exists()
    assert not (out_dir / 'shoulder_bracket.step').exists()


def test_keeps_the_parts_the_mechanism_does_have(out_dir):
    """A sweep that takes the live parts with it is worse than no sweep."""
    export.prune_stale(out_dir, keep_for('direct'))

    for name in ('base_plate', 'shoulder_turret'):
        assert (out_dir / f'{name}.stl').exists()
        assert (out_dir / f'{name}.step').exists()


def test_keeps_the_assembly(out_dir):
    """
    main() writes assembly.step AFTER the parts.

    It is absent while generate() runs, so a sweep that did not know
    about it would delete the previous run's copy on every build --
    harmless until a run fails partway and leaves none at all.
    """
    export.prune_stale(out_dir, keep_for('direct'))

    assert (out_dir / export.ASSEMBLY).exists()


def test_leaves_files_it_does_not_own(out_dir):
    """
    This directory is one people point slicers at and drop gcode in.

    Deleting someone's sliced output because it sat next to the STLs
    would be a far worse surprise than the stale file this sweep exists
    to remove.
    """
    (out_dir / 'base_plate.gcode').write_text(';FLAVOR:Marlin\n')

    removed = export.prune_stale(out_dir, keep_for('direct'))

    assert (out_dir / 'base_plate.gcode').exists()
    assert (out_dir / 'PRINTING.md').exists()
    assert 'base_plate.gcode' not in removed
    assert 'PRINTING.md' not in removed


def test_linkage_keeps_its_own_part(out_dir):
    """
    The same directory layout, swept for the other mechanism.

    shoulder_bracket is a real linkage part, so sweeping out/linkage/
    must not touch it. A sweep keyed to the wrong mechanism would.
    """
    assert 'shoulder_bracket' in parts_mod.parts_for('linkage')

    removed = export.prune_stale(out_dir, keep_for('linkage'))

    assert removed == []
    assert (out_dir / 'shoulder_bracket.stl').exists()


def test_a_clean_directory_is_left_alone(out_dir):
    """Running twice must be the same as running once."""
    export.prune_stale(out_dir, keep_for('direct'))

    assert export.prune_stale(out_dir, keep_for('direct')) == []
