#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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
Simple Bounding Box Question Annotator

Click and drag to create bounding boxes, then add questions for each box.
Results are saved to a YAML file.

Usage:
    python annotate.py path/to/image.png

Controls:
    - Click and drag: Create bounding box
    - 'd': Delete last box
    - 'r': Reset all boxes
    - 's': Save to YAML file
    - 'q' or ESC: Quit
"""

import argparse
from pathlib import Path
import sys
from typing import Any, Optional

import cv2
import numpy as np
import yaml


class BoundingBox:
    def __init__(self, x1: int, y1: int, x2: int, y2: int, question: str = ""):
        self.x1 = min(x1, x2)
        self.y1 = min(y1, y2)
        self.x2 = max(x1, x2)
        self.y2 = max(y1, y2)
        self.question = question

    def get_range(self) -> dict[str, list[int]]:
        return {"x": [self.x1, self.x2], "y": [self.y1, self.y2]}


class BBoxAnnotator:
    def __init__(self, image_path: str, yaml_path: str = "questions.yaml"):
        self.image_path = image_path
        self.yaml_path = yaml_path

        self.image = cv2.imread(image_path)
        if self.image is None:
            raise ValueError(f"Could not load image from {image_path}")

        # Create copies for drawing
        self.display_image = self.image.copy()
        self.original_image = self.image.copy()

        # Bounding boxes
        self.boxes: list[BoundingBox] = []
        self.current_box: BoundingBox | None = None

        # Drawing state
        self.drawing = False
        self.start_point: tuple[int, int] | None = None

        # Display settings
        self.window_name = "Bounding Box Annotator - Drag to create box"
        self.box_color = (0, 255, 0)  # Green
        self.current_box_color = (0, 165, 255)  # Orange

    def mouse_callback(self, event: int, x: int, y: int, flags: int, param: Any) -> None:
        """Handle mouse events for drawing bounding boxes"""
        # Adjust for info panel
        y_adjusted = y - 120 if y > 120 else 0

        if y < 120:  # Click is in info panel
            return

        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.start_point = (x, y_adjusted)

        elif event == cv2.EVENT_MOUSEMOVE:
            if self.drawing and self.start_point:
                self.current_box = BoundingBox(
                    self.start_point[0], self.start_point[1], x, y_adjusted
                )
                self.draw_all()

        elif event == cv2.EVENT_LBUTTONUP:
            if self.drawing and self.start_point:
                self.drawing = False

                final_box = BoundingBox(self.start_point[0], self.start_point[1], x, y_adjusted)

                # Only add if box has some size
                if abs(final_box.x2 - final_box.x1) > 5 and abs(final_box.y2 - final_box.y1) > 5:
                    question = self.get_question_input()
                    if question:
                        final_box.question = question
                        self.boxes.append(final_box)
                        print(f"\n✓ Added box {len(self.boxes)}")
                        print(
                            f"  Range: x:[{final_box.x1}, {final_box.x2}], y:[{final_box.y1}, {final_box.y2}]"
                        )
                        print(f"  Question: {question}\n")

                self.current_box = None
                self.start_point = None
                self.draw_all()

    def get_question_input(self) -> str:
        """Get question input from user via terminal"""
        print("\n" + "=" * 70)
        print("Enter your question for this bounding box:")
        print("=" * 70)

        try:
            question = input("Question: ").strip()
            return question
        except (EOFError, KeyboardInterrupt):
            print("\nSkipped")
            return ""

    def draw_all(self) -> None:
        """Draw all bounding boxes and UI"""
        self.display_image = self.original_image.copy()

        # Draw completed boxes
        for i, box in enumerate(self.boxes):
            cv2.rectangle(self.display_image, (box.x1, box.y1), (box.x2, box.y2), self.box_color, 3)

            # Draw number label with larger background
            label = f"{i + 1}"
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.8
            font_thickness = 2
            text_size = cv2.getTextSize(label, font, font_scale, font_thickness)[0]

            label_x = box.x1 + 5
            label_y = box.y1 + 30

            # Draw larger background
            cv2.rectangle(
                self.display_image,
                (label_x - 5, label_y - text_size[1] - 5),
                (label_x + text_size[0] + 5, label_y + 5),
                self.box_color,
                -1,
            )

            cv2.putText(
                self.display_image,
                label,
                (label_x, label_y),
                font,
                font_scale,
                (0, 0, 0),
                font_thickness,
            )

        # Draw current box being drawn
        if self.current_box:
            cv2.rectangle(
                self.display_image,
                (self.current_box.x1, self.current_box.y1),
                (self.current_box.x2, self.current_box.y2),
                self.current_box_color,
                3,
            )

        # Draw info panel
        self.draw_info_panel()

        cv2.imshow(self.window_name, self.display_image)

    def draw_info_panel(self) -> None:
        """Draw information panel on the image"""
        panel_height = 120
        panel = np.zeros((panel_height, self.display_image.shape[1], 3), dtype=np.uint8)

        # White background
        panel[:] = (240, 240, 240)

        # Add text to panel with bold font
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        font_thickness = 2
        text_color = (0, 0, 0)

        info_lines = [
            f"Boxes: {len(self.boxes)}  |  Image: {Path(self.image_path).name}",
            "DRAG mouse to create box, then enter question in terminal",
            "Controls:  [D] Delete last  [R] Reset  [S] Save  [Q] Quit",
        ]

        y_offset = 30
        for line in info_lines:
            cv2.putText(panel, line, (15, y_offset), font, font_scale, text_color, font_thickness)
            y_offset += 35

        # Combine panel with image
        self.display_image = np.vstack([panel, self.display_image])

    def save_to_yaml(self) -> None:
        """Save annotations to YAML file"""
        if not self.boxes:
            print("\n⚠ No boxes to save!")
            return

        # Prepare data structure
        questions_data: list[dict[str, Any]] = []
        for box in self.boxes:
            if box.question:
                questions_data.append(
                    {
                        "query": box.question,
                        "expected_range": {"x": [box.x1, box.x2], "y": [box.y1, box.y2]},
                    }
                )

        if not questions_data:
            print("\n⚠ No boxes with questions to save!")
            return

        # Save to file
        try:
            # Custom YAML formatting to get [x, y] format
            with open(self.yaml_path, "w") as f:
                f.write("questions:\n")
                for q in questions_data:
                    f.write(f'  - query: "{q["query"]}"\n')
                    f.write("    expected_range:\n")
                    x_range = q["expected_range"]["x"]
                    y_range = q["expected_range"]["y"]
                    f.write(f"      x: {x_range}\n")
                    f.write(f"      y: {y_range}\n")

            print(f"\n✓ Saved {len(questions_data)} question(s) to {self.yaml_path}")

        except Exception as e:
            print(f"\n✗ Error saving YAML: {e}")

    def delete_last_box(self) -> None:
        """Delete the last bounding box"""
        if self.boxes:
            self.boxes.pop()
            print("\n✓ Deleted last box")
            self.draw_all()
        else:
            print("\n⚠ No boxes to delete")

    def reset_boxes(self) -> None:
        """Reset all bounding boxes"""
        if self.boxes:
            self.boxes = []
            print("\n✓ All boxes cleared")
            self.draw_all()

    def run(self) -> None:
        """Run the interactive annotator"""
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        print("\n" + "=" * 70)
        print("BOUNDING BOX QUESTION ANNOTATOR")
        print("=" * 70)
        print(f"Image: {self.image_path}")
        print(f"Output: {self.yaml_path}")
        print()
        print("How to use:")
        print("  1. DRAG your mouse to create a bounding box")
        print("  2. Enter a question in the terminal")
        print("  3. Repeat for all questions")
        print("  4. Press 'S' to save to YAML file")
        print()
        print("Controls:")
        print("  D - Delete last box")
        print("  R - Reset all boxes")
        print("  S - Save to YAML")
        print("  Q - Quit")
        print("=" * 70 + "\n")

        self.draw_all()

        while True:
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q") or key == ord("Q") or key == 27:
                break
            elif key == ord("d") or key == ord("D"):
                self.delete_last_box()
            elif key == ord("r") or key == ord("R"):
                self.reset_boxes()
            elif key == ord("s") or key == ord("S"):
                self.save_to_yaml()

        # Prompt to save before exit
        if self.boxes:
            print("\n" + "=" * 70)
            print("Save annotations before exiting? (y/n): ", end="")
            try:
                response = input().strip().lower()
                if response == "y":
                    self.save_to_yaml()
            except (EOFError, KeyboardInterrupt):
                print("\nNot saved")

        cv2.destroyAllWindows()
        print("\nGoodbye!\n")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Simple bounding box annotator with questions",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
  python annotate.py floorplan.png
  python annotate.py floorplan.png --output my_questions.yaml
        """,
    )

    parser.add_argument("image", type=str, help="Path to the image file")

    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default="questions.yaml",
        help="Output YAML file (default: questions.yaml)",
    )

    args = parser.parse_args()

    # Check if image exists
    if not Path(args.image).exists():
        print(f"Error: Image file '{args.image}' not found")
        sys.exit(1)

    # Run annotator
    try:
        annotator = BBoxAnnotator(args.image, args.output)
        annotator.run()
    except Exception as e:
        print(f"Error: {e}")
        import traceback

        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
