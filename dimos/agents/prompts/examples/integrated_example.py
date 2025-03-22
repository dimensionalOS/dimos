"""Integrated example of prompt templates and builder.

This example demonstrates how to combine the PromptTemplates and PromptBuilder
classes to create complex, token-optimized prompts.
"""

from textwrap import dedent
from dimos.agents.prompts import PromptTemplates, PromptBuilder
from dimos.agents.prompts.examples.segmentation_example import create_segmentation_templates


def create_integrated_prompt(user_query, objects, target_objects, base64_image=None, 
                            image_width=None, image_height=None):
    """Creates a prompt using both templates and the builder.
    
    This function demonstrates how to create a segmentation template and then
    use the PromptBuilder to create a token-optimized prompt with the template
    content and optionally an image.
    
    Args:
        user_query: The user's query text
        objects: List of objects with their distances
        target_objects: List or string of objects to segment
        base64_image: Optional base64-encoded image
        image_width: Width of the image if provided
        image_height: Height of the image if provided
        
    Returns:
        A messages array ready to be sent to an LLM API
    """
    # Format object list
    if isinstance(objects, list):
        object_list_str = "\n".join([f"- {obj}" for obj in objects])
    else:
        object_list_str = objects
        
    # Format target objects
    if isinstance(target_objects, list):
        target_objects_str = ", ".join(target_objects)
    else:
        target_objects_str = target_objects
    
    # Create segmentation templates
    templates = create_segmentation_templates()
    
    # Create the segmentation prompt content
    segmentation_prompt = templates.create_combined_prompt(
        ["header", "object_list", "instructions", "output_format"],
        object_list=object_list_str,
        target_objects=target_objects_str,
        distance_unit="meters"
    )
    
    # Create the builder
    builder = PromptBuilder(model_name='gpt-4o')
    
    # Custom system prompt that includes our segmentation instructions
    system_prompt = dedent(f"""
    You are an AI vision assistant specialized in segmenting and analyzing images.
    
    {segmentation_prompt}

    Follow the instructions carefully and provide accurate segmentation results.
    """)
    
    # Build the final prompt with the builder
    messages = builder.build(
        system_prompt=system_prompt,
        user_query=user_query,
        base64_image=base64_image,
        image_width=image_width,
        image_height=image_height,
        image_detail="high" if base64_image else "low"
    )
    
    return messages


# Example usage
if __name__ == "__main__":
    example_objects = [
        "Chair (2.5m)",
        "Table (1.8m)", 
        "Laptop (0.5m)",
        "Coffee mug (0.3m)"
    ]
    
    user_question = "Can you segment the objects in this image and tell me the distance of each from the camera?"
    
    # Normally you would load a real base64 image here
    # For this example, we'll just use None
    messages = create_integrated_prompt(
        user_query=user_question,
        objects=example_objects,
        target_objects=["Chair", "Table", "Laptop"],
        base64_image=None
    )
    
    print("\nGenerated Messages Structure:")
    print("-" * 40)
    
    # Print system prompt
    print("System Prompt:")
    print(messages[0]["content"])
    print("\nUser Content:")
    print(messages[1]["content"][0]["text"])
    print("-" * 40) 