import sys
import os

# Add the parent directory of 'tests' to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

print(f"Hi from {os.path.basename(__file__)}\n")

# -----

from typing import List, Union, Dict

import dotenv
dotenv.load_dotenv()

import json
import requests
from textwrap import dedent
from openai import OpenAI, pydantic_function_tool
from pydantic import BaseModel, Field

MODEL = "gpt-4o-2024-08-06"

math_tutor_prompt = '''
    You are a helpful math tutor. You will be provided with a math problem,
    and your goal will be to output a step by step solution, along with a final answer.
    For each step, just provide the output as an equation use the explanation field to detail the reasoning.
'''

general_prompt = '''
    Follow the instructions. Output a step by step solution, along with a final answer. Use the explanation field to detail the reasoning.
'''

client = OpenAI()

class MathReasoning(BaseModel):
    class Step(BaseModel):
        explanation: str
        output: str

    steps: list[Step]
    final_answer: str

# region Function Calling
class GetWeather(BaseModel):
    latitude: str = Field(
        ...,
        description="latitude e.g. Bogotá, Colombia"
    )
    longitude: str = Field(
        ...,
        description="longitude e.g. Bogotá, Colombia"
    )

def get_weather(latitude, longitude):
    response = requests.get(f"https://api.open-meteo.com/v1/forecast?latitude={latitude}&longitude={longitude}&current=temperature_2m,wind_speed_10m&hourly=temperature_2m,relative_humidity_2m,wind_speed_10m&temperature_unit=fahrenheit")
    data = response.json()
    return data['current']['temperature_2m']

def get_tools():
    return [pydantic_function_tool(GetWeather)]
tools = get_tools()

def call_function(name, args):
    if name == "get_weather":
        print(f"Running function: {name}")
        print(f"Arguments are: {args}")
        return get_weather(**args)
    elif name == "GetWeather":
        print(f"Running function: {name}")
        print(f"Arguments are: {args}")
        return get_weather(**args)
    else:
        return f"Local function not found: {name}"
    
def callback(message, messages, response_message, tool_calls):
    if message is None or message.tool_calls is None:
        print("No message or tools were called.")
        return

    has_called_tools = False
    for tool_call in message.tool_calls:
        messages.append(response_message)

        has_called_tools = True
        name = tool_call.function.name
        args = json.loads(tool_call.function.arguments)

        result = call_function(name, args)
        print(f"Function Call Results: {result}")
        
        messages.append({
            "role": "tool",
            "tool_call_id": tool_call.id,
            "content": str(result),
            "name": name
        })
    
    # Complete the second call, after the functions have completed.
    if has_called_tools:
        print("Sending Second Query.")
        completion_2 = client.beta.chat.completions.parse(
            model=MODEL,
            messages=messages,
            response_format=MathReasoning,
            tools=tools,
        )
        print(f"Message: {completion_2.choices[0].message}")
        return completion_2.choices[0].message
    else:
        print("No Need for Second Query.")
        return None

# endregion Function Calling

def get_math_solution(question: str):
    prompt = general_prompt
    messages = [
            {"role": "system", "content": dedent(prompt)},
            {"role": "user", "content": question},
        ]
    response = client.beta.chat.completions.parse(
        model=MODEL,
        messages=messages, 
        response_format=MathReasoning,
        tools=tools
    )

    response_message = response.choices[0].message
    tool_calls = response_message.tool_calls

    new_response = callback(response.choices[0].message, messages, response_message, tool_calls)

    return new_response or response.choices[0].message

# Define Problem
problems = [
    "What is the derivative of 3x^2",
    "What's the weather like in San Fran today?"
]
problem = problems[0]

for problem in problems:
    print("================")
    print(f"Problem: {problem}")

    # Query for result
    solution = get_math_solution(problem)

    # If the query was refused
    if solution.refusal:
        print(f"Refusal: {solution.refusal}")
        break

    # If we were able to successfully parse the response back
    parsed_solution = solution.parsed
    if not parsed_solution:
        print(f"Unable to Parse Solution")
        print(f"Solution: {solution}")
        break
        
    # Print solution from class definitions
    print(f"Parsed: {parsed_solution}")

    steps = parsed_solution.steps
    print(f"Steps: {steps}")

    final_answer = parsed_solution.final_answer
    print(f"Final Answer: {final_answer}")
