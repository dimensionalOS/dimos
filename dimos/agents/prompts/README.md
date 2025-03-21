# Prompts Module

This module provides comprehensive tools for creating, managing, and optimizing prompts for AI models.

## Structure

The module is organized into the following submodules:

- **templates**: Classes for creating and managing reusable prompt templates
- **builder**: Classes for building token-optimized prompts for AI models
- **examples**: Example implementations showing how to use the module

## Templates

The `templates` submodule provides the `PromptTemplate` and `PromptTemplates` classes for creating, managing, and formatting prompt templates with parameter placeholders.

```python
from dimos.agents.prompts import PromptTemplate, PromptTemplates

# Create a collection of templates
templates = PromptTemplates()

# Add templates
templates.add_template("greeting", "Hello, {name}!")
templates.add_template("farewell", "Goodbye, {name}!")

# Format a single template
greeting = templates.format_prompt("greeting", name="Alice")
print(greeting)  # "Hello, Alice!"

# Combine multiple templates
combined = templates.create_combined_prompt(
    ["greeting", "farewell"], 
    name="Alice"
)
print(combined)  # "Hello, Alice!\n\nGoodbye, Alice!"
```

## Builder

The `builder` submodule provides the `PromptBuilder` class for building token-optimized prompts with support for multimodal content (text and images).

```python
from dimos.agents.prompts import PromptBuilder

# Create a builder
builder = PromptBuilder(model_name='gpt-4o')

# Build a basic prompt
messages = builder.build(
    user_query="What can you tell me about this image?",
    base64_image="...",  # Base64-encoded image
    image_width=800,
    image_height=600
)

# The result is ready to be sent to an LLM API
```

## Integration

The templates and builder can be used together to create powerful and flexible prompt generation systems:

```python
from dimos.agents.prompts import PromptTemplates, PromptBuilder

# Create templates
templates = PromptTemplates()
templates.add_template("instruction", "Analyze the following {subject}:")
templates.add_template("context", "The {subject} is related to {topic}.")

# Create formatted content
content = templates.create_combined_prompt(
    ["instruction", "context"],
    subject="image",
    topic="nature"
)

# Create a builder
builder = PromptBuilder()

# Build the final prompt
messages = builder.build(
    system_prompt=content,
    user_query="What do you see in this image?",
    base64_image="..."  # Base64-encoded image
)
```

## Examples

See the `examples` directory for complete examples:

- `segmentation_example.py`: Example of using templates for image segmentation
- `integrated_example.py`: Example of integrating templates and builder

## Use Cases

This module is useful for:

1. **Standardizing Prompts**: Create consistent prompt formats across your application
2. **Parameter Validation**: Ensure all required parameters are provided
3. **Token Optimization**: Keep prompts within token limits while prioritizing content
4. **Multimodal Content**: Support text and images in prompts
5. **Modular Design**: Mix and match prompt components for different scenarios 