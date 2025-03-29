# Copyright 2025 Dimensional Inc.
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

import inspect
from typing import Callable, Dict, Type
from pydantic import BaseModel, create_model
from dimos.robot.skills import AbstractSkill

class ROSSkillLibrary(AbstractSkill):
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(ROSSkillLibrary, cls).__new__(cls)
            cls._instance.skills = {}  # ROS Skills
            cls._instance.models = {}  # Pydantic models
        return cls._instance

    def register(self, name: str, func: Callable):
        """Registers a skill (function) into the library with a generated Pydantic model"""
        if name in self.skills:
            raise ValueError(f"Skill '{name}' is already registered!")
        self.skills[name] = func
        self.models[name] = self._generate_pydantic_model(name, func)

    def _generate_pydantic_model(self, name: str, func: Callable) -> Type[BaseModel]:
        """Dynamically generate a Pydantic model from the function signature"""
        signature = inspect.signature(func)
        fields = {}
        
        for param_name, param in signature.parameters.items():
            if param_name == "self":
                continue  # Skip 'self' for instance methods
            
            annotation = param.annotation if param.annotation != inspect.Parameter.empty else str
            default = param.default if param.default != inspect.Parameter.empty else ...
            fields[param_name] = (annotation, default)
        
        return create_model(name + "Model", **fields)

    def get_skill(self, name: str) -> Callable:
        """Retrieve a registered skill"""
        return self.skills.get(name, None)

    def get_model(self, name: str) -> Type[BaseModel]:
        """Retrieve the generated Pydantic model for validation"""
        return self.models.get(name, None)

    def list_skills(self) -> Dict[str, Callable]:
        """List all registered skills"""
        return self.skills
    
def register_skill(name: str = None):
    """Decorator to register a function into the SkillLibrary singleton"""
    def decorator(func: Callable):
        skill_name = name or func.__name__  # Default to function name if no name is given
        library = ROSSkillLibrary()
        #library.register(skill_name, func)

        def wrapper(instance, *args, **kwargs):
            """Wrapper that validates input arguments using Pydantic"""
            model = library.get_model(skill_name)  # Get the auto-generated Pydantic model
            if model:
                validated_args = model(**kwargs)  # Validate arguments with Pydantic
                return func(instance, **validated_args.model_dump())  # Call the function with validated args
            return func(instance, *args, **kwargs)  # If no model, call function normally

        return wrapper
    return decorator
