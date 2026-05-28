"""Tests for the multi-target VLM identification prompt + parser."""

from pawtrack.identify import (
    build_prompt,
    detect_all,
    detect_facing,
    parse_bboxes,
    parse_facing,
)


def test_parses_array_of_bbox_objects():
    text = '[{"bbox": [1, 2, 3, 4]}, {"bbox": [5, 6, 7, 8]}]'
    assert parse_bboxes(text) == [(1.0, 2.0, 3.0, 4.0), (5.0, 6.0, 7.0, 8.0)]


def test_parses_array_of_raw_boxes():
    assert parse_bboxes("[[1, 2, 3, 4]]") == [(1.0, 2.0, 3.0, 4.0)]


def test_parses_object_wrapping_a_list():
    assert parse_bboxes('{"objects": [{"bbox": [1, 2, 3, 4]}]}') == [
        (1.0, 2.0, 3.0, 4.0)]


def test_parses_a_single_bbox_object():
    # A model that answers with one detection object (not an array) must still
    # be parsed -- the dict itself is the box.
    assert parse_bboxes('{"bbox": [1, 2, 3, 4]}') == [(1.0, 2.0, 3.0, 4.0)]
    assert parse_bboxes('{"name": "a person", "bbox": [1, 2, 3, 4]}') == [
        (1.0, 2.0, 3.0, 4.0)]


def test_strips_code_fence_and_prose():
    fenced = '```json\n[{"bbox": [1, 2, 3, 4]}]\n```'
    assert parse_bboxes(fenced) == [(1.0, 2.0, 3.0, 4.0)]
    prose = 'Here are the boxes: [{"bbox": [1, 2, 3, 4]}] -- done.'
    assert parse_bboxes(prose) == [(1.0, 2.0, 3.0, 4.0)]


def test_drops_malformed_entries_keeps_valid():
    text = '[{"bbox": [1, 2, 3]}, {"bbox": [5, 6, 7, 8]}, "junk"]'
    assert parse_bboxes(text) == [(5.0, 6.0, 7.0, 8.0)]


def test_empty_or_none_yields_empty_list():
    assert parse_bboxes("") == []
    assert parse_bboxes("no people here") == []
    assert parse_bboxes("[]") == []


class _FakeVl:
    def __init__(self, response):
        self.response = response
        self.prompt = None

    def query(self, _image, prompt):
        self.prompt = prompt
        return self.response


def test_detect_all_queries_with_description_and_parses():
    vl = _FakeVl('[{"bbox": [1, 2, 3, 4]}]')
    boxes = detect_all(vl, object(), "a person sitting on a chair")
    assert boxes == [(1.0, 2.0, 3.0, 4.0)]
    assert "a person sitting on a chair" in vl.prompt


def test_build_prompt_includes_the_description():
    assert "a person in a red shirt" in build_prompt("a person in a red shirt")


def test_parse_facing_front_back_and_unknown():
    assert parse_facing("front") is True
    assert parse_facing("Front.") is True
    assert parse_facing("They are facing toward the camera") is True
    assert parse_facing("back") is False
    assert parse_facing("facing away") is False
    assert parse_facing("") is None  # empty
    assert parse_facing("not sure") is None  # neither cue
    assert parse_facing("front or back, hard to tell") is None  # both cues


def test_detect_facing_queries_and_parses():
    vl = _FakeVl("front")
    assert detect_facing(vl, object(), "a chair") is True
    assert "facing" in vl.prompt.lower()
    assert "a chair" in vl.prompt  # the prompt asks about the target object
    assert detect_facing(_FakeVl("back"), object(), "a chair") is False
