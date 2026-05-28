"""Multi-target identification: prompt a VL model for every match + parse it.

The single-box ``get_object_bbox_from_image`` grounding returns one box; the
greeter needs *all* matching subjects so it can pick one at random. This asks the
same VL model (via its duck-typed ``query(image, prompt)``) for a JSON list of
boxes and parses it leniently.

No DimOS import -- it works on any object with a ``query`` method and the parser
is pure, so both are unit tested without a model. The DimOS container injects the
real ``QwenChinaVlModel``.
"""

from __future__ import annotations

import json
import re

BBox = tuple[float, float, float, float]

_PROMPT = (
    "Look at this image and find EVERY {description}. Return ONLY a JSON array; "
    "each element is an object {{\"bbox\": [x1, y1, x2, y2]}} where x1,y1 is the "
    "top-left and x2,y2 the bottom-right corner in pixels. Return [] if there are "
    "none."
)


def build_prompt(description: str) -> str:
    """The multi-box detection prompt for a subject description."""
    return _PROMPT.format(description=description)


def _coerce_bbox(item: object) -> BBox | None:
    """Coerce one parsed element into a 4-float bbox, or None if it is not one."""
    if isinstance(item, dict):
        item = item.get("bbox") or item.get("box") or item.get("bbox_2d")
    if not isinstance(item, (list, tuple)) or len(item) != 4:
        return None
    try:
        x1, y1, x2, y2 = (float(v) for v in item)
    except (TypeError, ValueError):
        return None
    return (x1, y1, x2, y2)


def parse_bboxes(response_text: str) -> list[BBox]:
    """Parse a VL model's response into a list of bboxes (lenient).

    Accepts a bare JSON array (of ``{"bbox": [...]}`` objects or of raw
    ``[x1, y1, x2, y2]`` lists), or an object wrapping such an array, with or
    without ``` fences. Anything unparseable yields an empty list.

    Args:
        response_text: Raw model output.

    Returns:
        The valid 4-number boxes found, in order; empty if none.
    """
    if not response_text:
        return []
    text = response_text.strip()
    # Strip an optional ```json ... ``` code fence.
    fence = re.search(r"```(?:json)?\s*(.*?)```", text, re.DOTALL)
    if fence:
        text = fence.group(1).strip()
    # Fall back to the outermost JSON array/object if there is surrounding prose.
    if not text or text[0] not in "[{":
        match = re.search(r"[\[{].*[\]}]", text, re.DOTALL)
        if not match:
            return []
        text = match.group(0)
    try:
        parsed = json.loads(text)
    except json.JSONDecodeError:
        return []
    if isinstance(parsed, dict):
        if _coerce_bbox(parsed) is not None:
            # A single detection object, e.g. {"bbox": [x1, y1, x2, y2]} -- the
            # dict itself is one box (some models answer with one despite the
            # multi-box prompt).
            items: list[object] = [parsed]
        else:
            # A wrapper, e.g. {"objects": [...]} / {"boxes": [[...], ...]}: take
            # its first list of detections.
            items = next(
                (v for v in parsed.values() if isinstance(v, list)), []
            )
    elif isinstance(parsed, list):
        items = parsed
    else:
        return []
    boxes = [_coerce_bbox(item) for item in items]
    return [box for box in boxes if box is not None]


def detect_all(vl_model: object, image: object, description: str) -> list[BBox]:
    """Query ``vl_model`` for every ``description`` in ``image`` and parse boxes.

    Args:
        vl_model: Anything with ``query(image, prompt) -> str`` (e.g. a DimOS
            ``VlModel``).
        image: The frame to pass through to the model.
        description: Visual description of the subject (e.g. "a person sitting
            on a chair").

    Returns:
        Every parsed bounding box; empty if none were found or returned.
    """
    response = vl_model.query(image, build_prompt(description))
    return parse_bboxes(response)


def build_facing_prompt(description: str) -> str:
    """Prompt asking whether the target object faces the camera (front/back).

    Parameterised by the subject ``description`` so the same check works for
    whatever the greeter targets: a seated person on the robot, or a chair in
    simulation. A chair shows its backrest / is seen from behind when "facing
    away", so this lets the front/back logic (and the FingerHeart reward) be
    exercised in sim by pointing at chairs in different orientations.
    """
    return (
        "In this image, is {description} facing TOWARD the camera, or facing "
        "AWAY from it? A back / backrest / rear view, or seeing it from behind, "
        'counts as facing away. Answer with exactly one word: "front" if it '
        'faces the camera, "back" if it faces away.'
    ).format(description=description)


def parse_facing(response_text: str) -> bool | None:
    """Parse a front/back facing answer leniently.

    Returns True for a clear "front" (facing the camera), False for a clear
    "back" (facing away), and None when the answer mentions both or neither
    (orientation unknown) -- the caller picks a safe default.
    """
    if not response_text:
        return None
    low = response_text.strip().lower()
    front = "front" in low or "toward" in low or "towards" in low
    back = "back" in low or "away" in low or "behind" in low
    if front == back:  # both cues, or neither -> undecided
        return None
    return front


def detect_facing(
    vl_model: object, image: object, description: str
) -> bool | None:
    """Ask ``vl_model`` whether the targeted subject faces the camera.

    Args:
        vl_model: Anything with ``query(image, prompt) -> str``.
        image: The frame to pass through to the model.
        description: The subject being greeted (e.g. "a person sitting on a
            chair", or "a chair" in sim), so the prompt asks about that object.

    Returns:
        True if facing the camera, False if facing away, None if undecided.
    """
    return parse_facing(vl_model.query(image, build_facing_prompt(description)))
