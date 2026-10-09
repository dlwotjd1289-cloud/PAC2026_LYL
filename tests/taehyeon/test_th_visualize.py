import html
import xml.etree.ElementTree as ET

from th_helpers import make_box, make_context, make_state, placed
from pac_candidates import CandidateBackend
from virtual_data.policies import dblf_key
from virtual_data.visualize import decision_svg


def test_decision_svg_is_valid_and_complete():
    backend = CandidateBackend(make_context())
    state = make_state([placed("A", 0.002, 0.002, 0.0, weight=10), placed("B", 0.6, 0.002, 0.0, weight=1)])
    box = make_box("N", weight=5)
    cset = backend.candidate_set(box, state)
    chosen = min(cset.valid, key=dblf_key).candidate_id
    svg = decision_svg(state, box, cset, chosen)
    root = ET.fromstring(svg)  # well-formed XML
    assert root.tag.endswith("svg")
    assert html.escape(cset.summary()) in svg
    assert f"chosen {chosen}" in svg
    rects = [e for e in root.iter() if e.tag.endswith("rect")]
    # background + pallet + placed boxes + every candidate footprint (+ EMS)
    assert len(rects) >= 2 + len(state.pallet.boxes) + cset.generated_count
