"""Planning agent for breaking down tasks into executable steps.

This module provides a PlanningAgent that can:
- Engage in dialogue with users to understand tasks
- Break down tasks into concrete steps
- Refine plans based on user feedback
- Stream individual plan steps to other agents
"""

import threading
from typing import List, Optional, Dict
import json
from reactivex import Subject, Observable, disposable, create
from reactivex import operators as ops
from openai import OpenAI, NOT_GIVEN
import time
import logging
from dimos.robot.skills import AbstractSkill


from dimos.agents.agent import LLMAgent
from dimos.utils.logging_config import setup_logger

class PlanningAgent(LLMAgent):
    """Agent that plans and breaks down tasks through dialogue.
    
    This agent specializes in:
    1. Understanding complex tasks through dialogue
    2. Breaking tasks into concrete, executable steps
    3. Refining plans based on user feedback
    4. Streaming individual steps to other agents
    
    The agent maintains conversation state and can refine plans until
    the user confirms they are ready to execute.
    """
    
    def __init__(self,
                 dev_name: str = "PlanningAgent",
                 model_name: str = "gpt-4",
                 max_steps: int = 10,
                 input_query_stream: Optional[Observable] = None,
                 use_terminal: bool = False,
                 skills: Optional[AbstractSkill] = None):
        """Initialize the planning agent.
        
        Args:
            dev_name: Name identifier for the agent
            model_name: OpenAI model to use
            max_steps: Maximum number of steps in a plan
            input_query_stream: Observable stream of user queries
            use_terminal: Whether to enable terminal input
        """

        self.skills = skills
        skills_list = []
        if self.skills is not None:
            print(type(self.skills.get_tools()))
            skills_list = (self.skills.get_tools())
            print(skills_list)
        
        
        self.system_prompt = f"""You are a Robot planning assistant that helps break down tasks into concrete, executable steps.
Your goal is to:
1. Break down the task into clear, sequential steps
2. Refine the plan based on user feedback as needed
3. Only finalize the plan when the user explicitly confirms

You have the following skills at your disposal:
{skills_list}

IMPORTANT: You MUST ALWAYS respond with ONLY valid JSON in the following format, with no additional text or explanation:
{{
    "type": "dialogue" | "plan",
    "content": string | list[string],
    "needs_confirmation": boolean
}}

Your goal is to:
1. Understand the user's task through dialogue
2. Break it down into clear, sequential steps
3. Refine the plan based on user feedback
4. Only finalize the plan when the user explicitly confirms

For dialogue responses, use:
{{
    "type": "dialogue",
    "content": "Your message to the user",
    "needs_confirmation": false
}}

For plan proposals, use:
{{
    "type": "plan",
    "content": ["Execute", "Execute", ...],
    "needs_confirmation": true
}}

Remember: ONLY output valid JSON, no other text."""
        
        super().__init__(dev_name=dev_name, agent_type="Planning")
        
        # Set logger to debug level
        self.logger.setLevel(logging.DEBUG)
        self.logger.debug("Setting up PlanningAgent with debug logging enabled")
        
        # OpenAI client
        self.client = OpenAI()
        self.model_name = model_name
        self.max_steps = max_steps
        
        # Reduce token limits to avoid context length errors
        self.max_output_tokens_per_request = 1000
        
        # Planning state
        self.conversation_history = []
        self.current_plan = []
        self.plan_confirmed = False
        
        # Latest response for terminal mode
        self.latest_response = None
        
        # Set up query stream subscription if provided
        self.input_query_stream = input_query_stream
        if self.input_query_stream:
            self.logger.info("Setting up query stream subscription")
            self.disposables.add(self.subscribe_to_query_processing(self.input_query_stream))
            
        # Terminal mode
        self.use_terminal = use_terminal

        if use_terminal:
            # Start terminal interface in a separate thread
            terminal_thread = threading.Thread(target=self.start_terminal_interface, daemon=True)
            terminal_thread.start()
            self.logger.info("Terminal interface started in separate thread")
        
        self.logger.info("Planning agent initialized")
        
    def _send_query(self, messages: list) -> dict:
        """Send query to OpenAI and parse the response.
        
        Args:
            messages: List of message dictionaries
            
        Returns:
            dict: Parsed response with type, content, and needs_confirmation
        """
        try:
            response = self.client.chat.completions.create(
                model=self.model_name,
                messages=messages,
                max_tokens=self.max_output_tokens_per_request
            )
            
            # Parse response as JSON
            response_text = response.choices[0].message.content
            return json.loads(response_text)
            
        except Exception as e:
            self.logger.error(f"Error in _send_query: {e}")
            return {
                "type": "dialogue",
                "content": f"I encountered an error: {str(e)}",
                "needs_confirmation": False
            }

    def _handle_response(self, response: dict) -> None:
        """Handle the agent's response and update state.
        
        Args:
            response: Parsed response dictionary
        """
        # Add to conversation history
        self.conversation_history.append(response)
        
        # If it's a plan, update current plan
        if response["type"] == "plan":
            self.logger.info(f"Updating current plan: {response['content']}")
            self.current_plan = response["content"]
            
        # Store latest response
        self.latest_response = response
            
        # Emit response to observers
        # self.response_subject.on_next(response)

    def _stream_plan(self) -> None:
        """Stream each step of the confirmed plan."""
        self.logger.info("Starting to stream plan steps")
        self.logger.debug(f"Current plan: {self.current_plan}")

        for i, step in enumerate(self.current_plan, 1):
            self.logger.info(f"Streaming step {i}: {step}")
            # Add a small delay between steps to ensure they're processed
            time.sleep(0.5)
            try:
                self.response_subject.on_next(str(step))
                self.logger.debug(f"Successfully emitted step {i} to response_subject")
            except Exception as e:
                self.logger.error(f"Error emitting step {i}: {e}")
        self.logger.info("Plan streaming completed")
        self.response_subject.on_completed()  # Complete the observable after all steps

    def process_user_input(self, user_input: str) -> None:
        """Process user input and generate appropriate response.
        
        Args:
            user_input: The user's message
        """
        if not user_input:
            return
            
        # Check for plan confirmation
        if self.current_plan and user_input.lower() in ["yes", "y", "confirm"]:
            self.logger.info("Plan confirmation received")
            self.plan_confirmed = True
            confirmation_msg = {
                "type": "dialogue",
                "content": "Plan confirmed! Streaming steps to execution...",
                "needs_confirmation": False
            }
            self._handle_response(confirmation_msg)
            self._stream_plan()
            return
            
        # Build messages for OpenAI
        messages = [
            {"role": "system", "content": self.system_prompt}
        ]
        
        # Add the new user input to conversation history
        self.conversation_history.append({
            "type": "user_message",
            "content": user_input
        })
        
        # Add complete conversation history including both user and assistant messages
        for msg in self.conversation_history:
            if msg["type"] == "user_message":
                messages.append({"role": "user", "content": msg["content"]})
            elif msg["type"] == "dialogue":
                messages.append({"role": "assistant", "content": msg["content"]})
            elif msg["type"] == "plan":
                # For plans, format them nicely in the conversation
                plan_text = "Here's my proposed plan:\n" + "\n".join(f"{i+1}. {step}" for i, step in enumerate(msg["content"]))
                messages.append({"role": "assistant", "content": plan_text})
        
        # Get and handle response
        response = self._send_query(messages)
        self._handle_response(response)

    def start_terminal_interface(self):
        """Start the terminal interface for input/output."""
        print("\nWelcome to the Planning Assistant!")
        print("Describe your task and I'll help break it down into steps.")
        print("Once you're happy with the plan, type 'yes' to execute it.")
        print("Type 'quit' to exit.\n")
        
        while True:
            try:
                user_input = input("> ")
                if user_input.lower() in ['quit', 'exit']:
                    break
                    
                self.process_user_input(user_input)
                
                # Display response
                if self.latest_response["type"] == "dialogue":
                    print(f"\nPlanner: {self.latest_response['content']}")
                elif self.latest_response["type"] == "plan":
                    print("\nProposed Plan:")
                    for i, step in enumerate(self.latest_response["content"], 1):
                        print(f"{i}. {step}")
                    if self.latest_response["needs_confirmation"]:
                        print("\nDoes this plan look good? (yes/no)")
                        
                if self.plan_confirmed:
                    print("\nPlan confirmed! Streaming steps to execution...")
                    break
                    
            except KeyboardInterrupt:
                print("\nStopping...")
                break
            except Exception as e:
                print(f"\nError: {e}")
                break

    def get_plan_observable(self) -> Observable:
        """Get an observable that emits plan steps when confirmed.
        
        Returns:
            Observable: Stream of individual plan steps
        """
        self.logger.info("Creating plan observable")
        return self.get_response_observable()

    # def dispose_all(self):
    #     """Clean up resources."""
    #     self.logger.info("Disposing planning agent resources")
    #     super().dispose_all()
    #     self.response_subject.on_completed()
