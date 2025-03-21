"""Template implementation for creating and managing AI prompt templates.

This module provides classes for creating, managing, and formatting prompt templates
for AI agents. It supports defining templates with parameter placeholders, validating
required parameters, and combining multiple templates into a single prompt.
"""

from typing import Dict, Any, Optional, List, Union
import string
import re


class PromptTemplate:
    """A template for an AI prompt that can be formatted with parameters.
    
    This class represents a single prompt template with placeholders for parameters.
    It supports parameter validation and formatting.
    
    Attributes:
        template: A string template with placeholders for parameters.
        required_params: A list of parameter names required by this template.
    
    Example:
        >>> template = PromptTemplate("Hello, {name}!")
        >>> message = template.format(name="World")
        >>> print(message)
        "Hello, World!"
    """
    
    def __init__(self, template: str, required_params: Optional[List[str]] = None) -> None:
        """Initializes a PromptTemplate with a template string.
        
        Args:
            template: A string with {parameter} placeholders.
            required_params: Optional list of required parameter names.
                If not provided, parameters are extracted from the template.
        """
        self.template = template
        
        # Extract parameter names from the template using regex
        pattern = r'\{([^{}]*)\}'
        extracted_params = re.findall(pattern, template)
        
        # If required_params is provided, use that, otherwise use extracted params
        self.required_params = required_params if required_params is not None else extracted_params
    
    def format(self, **kwargs) -> str:
        """Formats the template with the provided parameters.
        
        Args:
            **kwargs: The parameters to format the template with.
            
        Returns:
            The formatted prompt string.
            
        Raises:
            ValueError: If required parameters are missing.
        
        Example:
            >>> template = PromptTemplate("Hello, {name}!")
            >>> template.format(name="World")
            'Hello, World!'
        """
        # Check for missing required parameters
        missing_params = [param for param in self.required_params if param not in kwargs]
        if missing_params:
            raise ValueError(f"Missing required parameters: {', '.join(missing_params)}")
        
        # Format the template with provided parameters
        try:
            return self.template.format(**kwargs)
        except KeyError as e:
            raise ValueError(f"Missing parameter in template: {e}")


class PromptTemplates:
    """A collection of prompt templates for creating prompts for AI agents.
    
    This class manages multiple templates and provides methods to format them
    with various parameters to create prompts for models like OpenAI or Claude.
    
    Attributes:
        templates: A dictionary mapping template names to PromptTemplate objects.
    
    Example:
        >>> templates = PromptTemplates()
        >>> templates.add_template("greeting", "Hello, {name}!")
        >>> templates.add_template("farewell", "Goodbye, {name}!")
        >>> greeting = templates.format_prompt("greeting", name="Alice")
        >>> farewell = templates.format_prompt("farewell", name="Alice")
        >>> combined = templates.create_combined_prompt(
        ...     ["greeting", "farewell"], 
        ...     name="Alice"
        ... )
    """
    
    def __init__(self) -> None:
        """Initializes an empty collection of prompt templates."""
        self.templates: Dict[str, PromptTemplate] = {}
    
    def add_template(self, 
                     name: str, 
                     template: Union[str, PromptTemplate], 
                     required_params: Optional[List[str]] = None) -> None:
        """Adds a prompt template to the collection.
        
        Args:
            name: A unique identifier for the template.
            template: Either a string template or a PromptTemplate object.
            required_params: Optional list of required parameter names 
                (used only if template is a string).
        
        Example:
            >>> templates = PromptTemplates()
            >>> templates.add_template("greeting", "Hello, {name}!")
            >>> templates.add_template(
            ...     "intro", 
            ...     "I'm {assistant_name}, an AI assistant.",
            ...     required_params=["assistant_name", "version"]
            ... )
        """
        if isinstance(template, str):
            template = PromptTemplate(template, required_params)
        
        self.templates[name] = template
    
    def remove_template(self, name: str) -> None:
        """Removes a template from the collection.
        
        Args:
            name: The name of the template to remove.
            
        Raises:
            KeyError: If the template doesn't exist.
        
        Example:
            >>> templates = PromptTemplates()
            >>> templates.add_template("greeting", "Hello, {name}!")
            >>> templates.remove_template("greeting")
        """
        if name not in self.templates:
            raise KeyError(f"Template '{name}' not found.")
        
        del self.templates[name]
    
    def get_template(self, name: str) -> PromptTemplate:
        """Gets a prompt template by name.
        
        Args:
            name: The name of the template.
            
        Returns:
            The PromptTemplate object.
            
        Raises:
            KeyError: If the template doesn't exist.
        
        Example:
            >>> templates = PromptTemplates()
            >>> templates.add_template("greeting", "Hello, {name}!")
            >>> template = templates.get_template("greeting")
            >>> print(template.template)
            'Hello, {name}!'
        """
        if name not in self.templates:
            raise KeyError(f"Template '{name}' not found.")
        
        return self.templates[name]
    
    def format_prompt(self, template_name: str, **kwargs) -> str:
        """Formats a prompt template with the provided parameters.
        
        Args:
            template_name: The name of the template to format.
            **kwargs: The parameters to format the template with.
            
        Returns:
            The formatted prompt string.
            
        Raises:
            KeyError: If the template doesn't exist.
            ValueError: If required parameters are missing.
        
        Example:
            >>> templates = PromptTemplates()
            >>> templates.add_template("greeting", "Hello, {name}!")
            >>> greeting = templates.format_prompt("greeting", name="Alice")
            >>> print(greeting)
            'Hello, Alice!'
        """
        if template_name not in self.templates:
            raise KeyError(f"Template '{template_name}' not found.")
        
        return self.templates[template_name].format(**kwargs)
    
    def create_combined_prompt(self, template_names: List[str], **kwargs) -> str:
        """Creates a prompt by combining multiple templates.
        
        This combines templates in the order provided and formats them with 
        the provided parameters.
        
        Args:
            template_names: A list of template names to combine.
            **kwargs: The parameters to format the templates with.
            
        Returns:
            The combined and formatted prompt string.
            
        Raises:
            KeyError: If any template doesn't exist.
            ValueError: If required parameters are missing.
        
        Example:
            >>> templates = PromptTemplates()
            >>> templates.add_template("greeting", "Hello, {name}!")
            >>> templates.add_template("intro", "I'm {assistant_name}.")
            >>> combined = templates.create_combined_prompt(
            ...     ["greeting", "intro"], 
            ...     name="Alice", 
            ...     assistant_name="Claude"
            ... )
            >>> print(combined)
            'Hello, Alice!
            
            I'm Claude.'
        """
        missing_templates = [name for name in template_names if name not in self.templates]
        if missing_templates:
            raise KeyError(f"Templates not found: {', '.join(missing_templates)}")
        
        formatted_prompts = [
            self.format_prompt(name, **kwargs) for name in template_names
        ]
        
        # Join the formatted prompts with double newlines
        return "\n\n".join(formatted_prompts) 