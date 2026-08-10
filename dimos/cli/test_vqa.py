# Copyright 2026 Dimensional Inc.

from dimos.cli import vqa


def test_question_mode_selects_the_image_author_without_an_explicit_query() -> None:
    assert vqa._uses_image_question_author("constrained", []) is True
    assert vqa._uses_image_question_author("constrained", ["chair"]) is False
    assert vqa._uses_image_question_author("agentic", ["chair"]) is True
