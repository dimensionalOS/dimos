import threading
from typing import List, Optional, Dict, Union, Literal
import json
from reactivex import Subject, Observable, disposable, create
from reactivex import operators as ops
from openai import OpenAI, NOT_GIVEN
import time
import logging
from dimos.robot.skills import AbstractSkill
from dimos.agents.agent import OpenAIAgent
from dimos.utils.logging_config import setup_logger
from textwrap import dedent
from pydantic import BaseModel, Field

# For response validation
class PlanningAgentResponse(BaseModel):
    type: Literal["dialogue", "plan"]
    content: List[str]
    needs_confirmation: bool

class PlanningAgent(OpenAIAgent):
    """Agent that plans and breaks down tasks through dialogue.
    
    This agent specializes in:
    1. Understanding complex tasks through dialogue
    2. Breaking tasks into concrete, executable steps
    3. Refining plans based on user feedback
    4. Streaming individual steps to ExecutionAgents
    
    The agent maintains conversation state and can refine plans until
    the user confirms they are ready to execute.
    """
    
    def __init__(self,
                 dev_name: str = "PlanningAgent",
                 model_name: str = "gpt-4",
                 input_query_stream: Optional[Observable] = None,
                 use_terminal: bool = False,
                 skills: Optional[AbstractSkill] = None):
        """Initialize the planning agent.
        
        Args:
            dev_name: Name identifier for the agent
            model_name: OpenAI model to use
            input_query_stream: Observable stream of user queries
            use_terminal: Whether to enable terminal input
            skills: Available skills/functions for the agent
        """
        # Planning state
        self.conversation_history = []
        self.current_plan = []
        self.plan_confirmed = False
        self.latest_response = None
        
        # Build system prompt
        skills_list = []
        if skills is not None:
            skills_list = skills.get_tools()
        
        system_query = dedent(f"""
            You are a Robot planning assistant that helps break down tasks into concrete, executable steps.
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

            Remember: ONLY output valid JSON, no other text.""")

        # Initialize OpenAIAgent with our configuration
        super().__init__(
            dev_name=dev_name,
            agent_type="Planning",
            query="",  # Will be set by process_user_input
            model_name=model_name,
            input_query_stream=input_query_stream,
            system_query=system_query,
            max_output_tokens_per_request=1000,
            response_model=PlanningAgentResponse
        )
        self.logger.info("Planning agent initialized")

        # Set up terminal mode if requested
        self.use_terminal = use_terminal
        if use_terminal:
            # Start terminal interface in a separate thread
            self.logger.info("Starting terminal interface in a separate thread")
            terminal_thread = threading.Thread(target=self.start_terminal_interface, daemon=True)
            terminal_thread.start()
            
    def _handle_response(self, response) -> None:
        """Handle the agent's response and update state.
        
        Args:
            response: ParsedChatCompletionMessage containing PlanningAgentResponse
        """
        print("handle response", response)
        print("handle response type", type(response))
        
        # Extract the PlanningAgentResponse from parsed field if available
        planning_response = response.parsed if hasattr(response, 'parsed') else response
        print("planning response", planning_response)
        print("planning response type", type(planning_response))
        # Convert to dict for storage in conversation history
        response_dict = planning_response.model_dump()
        self.conversation_history.append(response_dict)
        
        # If it's a plan, update current plan
        if planning_response.type == "plan":
            self.logger.info(f"Updating current plan: {planning_response.content}")
            self.current_plan = planning_response.content
            
        # Store latest response
        self.latest_response = response_dict
            

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
        self.response_subject.on_completed()
    
    def _send_query(self, messages: list) -> PlanningAgentResponse:
        """Send query to OpenAI and parse the response.
        
        Extends OpenAIAgent's _send_query to handle planning-specific response formats.
        
        Args:
            messages: List of message dictionaries
            
        Returns:
            PlanningAgentResponse: Validated response with type, content, and needs_confirmation
        """
        
        try:
            return super()._send_query(messages)
        except Exception as e:
            self.logger.error(f"Caught exception in _send_query: {str(e)}")
            return PlanningAgentResponse(
                type="dialogue",
                content=f"Error: {str(e)}",
                needs_confirmation=False
            )

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
            confirmation_msg = PlanningAgentResponse(
                type="dialogue",
                content="Plan confirmed! Streaming steps to execution...",
                needs_confirmation=False
            )
            self._handle_response(confirmation_msg)
            self._stream_plan()
            return
            
        # Build messages for OpenAI with conversation history
        messages = [
            {"role": "system", "content": self.system_query}  # Using system_query from OpenAIAgent
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
                plan_text = "Here's my proposed plan:\n" + "\n".join(f"{i+1}. {step}" for i, step in enumerate(msg["content"]))
                messages.append({"role": "assistant", "content": plan_text})
        
        # Get and handle response
        response = self._send_query(messages)
        self._handle_response(response)

    def start_terminal_interface(self):
        """Start the terminal interface for input/output."""

        time.sleep(5) # buffer time for clean terminal interface printing
        print("=" * 50)
        print("\nDimOS Action PlanningAgent\n")
        print("I have access to your Robot() and Robot Skills()")
        print("Describe your task and I'll break it down into steps using your skills as a reference.")
        print("Once you're happy with the plan, type 'yes' to execute it.")
        print("Type 'quit' to exit.\n")
        
        while True:
            try:
                print("=" * 50)
                user_input = input("USER > ")
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
    
    def get_response_observable(self) -> Observable:
        """Gets an observable that emits responses from this agent.

        This method processes the response stream from the parent class,
        extracting content from `PlanningAgentResponse` objects and flattening
        any lists of plan steps for emission.
        
        Returns:
            Observable: An observable that emits plan steps from the agent.
        """
        def extract_content(response) -> List[str]:
            if isinstance(response, PlanningAgentResponse):
                if response.type == "plan":
                    return response.content  # List of steps to be emitted individually
                else:  # dialogue type
                    return [response.content]  # Wrap single dialogue message in a list
            else:
                return [str(response)]  # Wrap non-PlanningAgentResponse in a list

        # Get base observable from parent class
        base_observable = super().get_response_observable()

        # Process the stream: extract content and flatten plan lists
        return base_observable.pipe(
            ops.map(extract_content),
            ops.flat_map(lambda items: items)  # Flatten the list of items
        )
