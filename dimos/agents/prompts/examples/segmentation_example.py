"""Example of segmentation prompt templates.

This file demonstrates how to create and use prompt templates for image segmentation tasks
using the PromptTemplate and PromptTemplates classes.
"""

from dimos.agents.prompts import PromptTemplates


def create_segmentation_templates():
    """Creates and returns a collection of templates for segmentation tasks."""
    
    # Create a collection of templates for segmentation tasks
    segmentation_templates = PromptTemplates()

    # Add templates for different parts of the segmentation prompt
    segmentation_templates.add_template(
        "header",
        "You have been provided an image of the space. You have a list of the following objects and their corresponding distances:"
    )

    segmentation_templates.add_template(
        "object_list",
        "Objects:\n{object_list}"
    )

    segmentation_templates.add_template(
        "instructions",
        "Please identify and segment the following objects in the image:\n{target_objects}"
    )

    segmentation_templates.add_template(
        "output_format",
        "For each identified object, provide:\n"
        "1. Bounding box coordinates (x1, y1, x2, y2)\n"
        "2. Segmentation mask\n"
        "3. Confidence score\n"
        "4. Object class\n"
        "5. Estimated distance: {distance_unit}"
    )

    segmentation_templates.add_template(
        "additional_guidance",
        "Additional guidance:\n"
        "- {guidance_point_1}\n"
        "- {guidance_point_2}\n"
        "- Pay attention to partially occluded objects\n"
        "- {guidance_point_3}"
    )
    
    return segmentation_templates


def create_example_segmentation_prompt():
    """Creates an example segmentation prompt using the templates."""
    
    # Get the segmentation templates
    segmentation_templates = create_segmentation_templates()
    
    # Example data
    example_objects = [
        "Chair (2.5m)",
        "Table (1.8m)",
        "Laptop (0.5m)",
        "Coffee mug (0.3m)",
        "Plant (3.1m)"
    ]
    
    object_list_str = "\n".join([f"- {obj}" for obj in example_objects])
    target_objects = "Chair, Table, Laptop"
    
    # Create the combined prompt
    complete_prompt = segmentation_templates.create_combined_prompt(
        ["header", "object_list", "instructions", "output_format", "additional_guidance"],
        object_list=object_list_str,
        target_objects=target_objects,
        distance_unit="meters",
        guidance_point_1="Focus on the spatial relationships between objects",
        guidance_point_2="Report any ambiguity in object identification",
        guidance_point_3="Use consistent labeling across all identified objects"
    )
    
    return complete_prompt


def create_custom_segmentation_prompt(objects, target_objects, distance_unit="meters", guidance=None):
    """Creates a custom segmentation prompt with provided objects and targets.
    
    Args:
        objects: List of objects with their distances (e.g., ["Chair (2.5m)", "Table (1.8m)"])
        target_objects: String or list of objects to segment
        distance_unit: Unit of measurement for distances (default: "meters")
        guidance: Optional dictionary with guidance points (keys: guidance_point_1, etc.)
    
    Returns:
        A formatted prompt string ready for use with AI models
    """
    segmentation_templates = create_segmentation_templates()
    
    # Format the object list
    if isinstance(objects, list):
        object_list_str = "\n".join([f"- {obj}" for obj in objects])
    else:
        object_list_str = objects
    
    # Format the target objects
    if isinstance(target_objects, list):
        target_objects_str = ", ".join(target_objects)
    else:
        target_objects_str = target_objects
    
    # Set default guidance if not provided
    guidance = guidance or {
        "guidance_point_1": "Focus on the spatial relationships between objects",
        "guidance_point_2": "Report any ambiguity in object identification",
        "guidance_point_3": "Use consistent labeling across all identified objects"
    }
    
    # Create the combined prompt
    parameters = {
        "object_list": object_list_str,
        "target_objects": target_objects_str,
        "distance_unit": distance_unit,
        **guidance
    }
    
    return segmentation_templates.create_combined_prompt(
        ["header", "object_list", "instructions", "output_format", "additional_guidance"],
        **parameters
    )


# Example usage
if __name__ == "__main__":
    prompt = create_example_segmentation_prompt()
    print("\nGenerated Segmentation Prompt:")
    print("-" * 40)
    print(prompt)
    print("-" * 40)
    
    # Example of custom prompt
    custom_objects = [
        "Bookshelf (3.2m)",
        "Desk (1.5m)",
        "Monitor (0.8m)",
        "Keyboard (0.4m)"
    ]
    
    custom_prompt = create_custom_segmentation_prompt(
        objects=custom_objects,
        target_objects=["Bookshelf", "Monitor"],
        guidance={
            "guidance_point_1": "Pay attention to the monitor's orientation",
            "guidance_point_2": "Note if there are books on the bookshelf",
            "guidance_point_3": "Report any cables connecting devices"
        }
    )
    
    print("\nCustom Segmentation Prompt:")
    print("-" * 40)
    print(custom_prompt)
    print("-" * 40) 