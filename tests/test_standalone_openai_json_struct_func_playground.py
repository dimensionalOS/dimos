import sys
import os

# Add the parent directory of 'tests' to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

print(f"Hi from {os.path.basename(__file__)}\n")

# -----
# # Milestone 1


# from typing import List, Dict, Optional
# import requests
# import json
# from pydantic import BaseModel, Field
# from openai import OpenAI, pydantic_function_tool

# # Environment setup
# import dotenv
# dotenv.load_dotenv()

# # Constants and prompts
# MODEL = "gpt-4o-2024-08-06"
# GENERAL_PROMPT = '''
#     Follow the instructions. Output a step by step solution, along with a final answer.
#     Use the explanation field to detail the reasoning.
# '''

# # Initialize OpenAI client
# client = OpenAI()

# # Models and functions
# class Step(BaseModel):
#     explanation: str
#     output: str

# class MathReasoning(BaseModel):
#     steps: List[Step]
#     final_answer: str

# class GetWeather(BaseModel):
#     latitude: str = Field(..., description="Latitude e.g., Bogotá, Colombia")
#     longitude: str = Field(..., description="Longitude e.g., Bogotá, Colombia")

# def fetch_weather(latitude: str, longitude: str) -> Dict:
#     url = f"https://api.open-meteo.com/v1/forecast?latitude={latitude}&longitude={longitude}&current=temperature_2m,wind_speed_10m&hourly=temperature_2m,relative_humidity_2m,wind_speed_10m&temperature_unit=fahrenheit"
#     response = requests.get(url)
#     return response.json().get('current', {})

# # Tool management
# def get_tools() -> List[BaseModel]:
#     return [pydantic_function_tool(GetWeather)]

# def handle_function_call(tool_call: Dict) -> Optional[str]:
#     if tool_call['name'] == "get_weather":
#         result = fetch_weather(**tool_call['args'])
#         return f"Temperature is {result['temperature_2m']}°F"
#     return None

# # Communication and processing with OpenAI
# def process_message_with_openai(question: str) -> MathReasoning:
#     messages = [
#         {"role": "system", "content": GENERAL_PROMPT.strip()},
#         {"role": "user", "content": question}
#     ]
#     response = client.beta.chat.completions.parse(
#         model=MODEL,
#         messages=messages,
#         response_format=MathReasoning,
#         tools=get_tools()
#     )
#     return response.choices[0].message

# def get_math_solution(question: str) -> MathReasoning:
#     solution = process_message_with_openai(question)
#     return solution

# # Example usage
# def main():
#     problems = [
#         "What is the derivative of 3x^2",
#         "What's the weather like in San Francisco today?"
#     ]
#     problem = problems[1]
#     print(f"Problem: {problem}")

#     solution = get_math_solution(problem)
#     if not solution:
#         print("Failed to get a solution.")
#         return

#     if not solution.parsed:
#         print("Failed to get a parsed solution.")
#         print(f"Solution: {solution}")
#         return

#     print(f"Steps: {solution.parsed.steps}")
#     print(f"Final Answer: {solution.parsed.final_answer}")

# if __name__ == "__main__":
#     main()


# # Milestone 1

# Milestone 2
import json
import os
import requests

from dotenv import load_dotenv
load_dotenv()

from openai import OpenAI

client = OpenAI()

def get_current_weather(latitude, longitude):
    """Get the current weather in a given latitude and longitude using the 7Timer API"""
    base = "http://www.7timer.info/bin/api.pl"
    request_url = f"{base}?lon={longitude}&lat={latitude}&product=civillight&output=json"
    response = requests.get(request_url)
    
    # Parse response to extract the main weather data
    weather_data = response.json()
    current_data = weather_data.get('dataseries', [{}])[0]
    
    result = {
        "latitude": latitude,
        "longitude": longitude,
        "temp": current_data.get('temp2m', {'max': 'Unknown', 'min': 'Unknown'}),
        "humidity": "Unknown"
    }
    
    # Convert the dictionary to JSON string to match the given structure
    return json.dumps(result)

def run_conversation(content):
    messages = [{"role": "user", "content": content}]
    tools = [
        {
            "type": "function",
            "function": {
                "name": "get_current_weather",
                "description": "Get the current weather in a given latitude and longitude",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "latitude": {
                            "type": "string",
                            "description": "The latitude of a place",
                        },
                        "longitude": {
                            "type": "string",
                            "description": "The longitude of a place",
                        },
                    },
                    "required": ["latitude", "longitude"],
                },
            },
        }
    ]
    response = client.chat.completions.create(
        model="gpt-3.5-turbo-0125",
        messages=messages,
        tools=tools,
        tool_choice="auto",
    )
    response_message = response.choices[0].message
    tool_calls = response_message.tool_calls

    if tool_calls:
        messages.append(response_message)

        available_functions = {
            "get_current_weather": get_current_weather,
        }
        for tool_call in tool_calls:
            print(f"Function: {tool_call.function.name}")
            print(f"Params:{tool_call.function.arguments}")
            function_name = tool_call.function.name
            function_to_call = available_functions[function_name]
            function_args = json.loads(tool_call.function.arguments)
            function_response = function_to_call(
                latitude=function_args.get("latitude"),
                longitude=function_args.get("longitude"),
            )
            print(f"API: {function_response}")
            messages.append(
                {
                    "tool_call_id": tool_call.id,
                    "role": "tool",
                    "name": function_name,
                    "content": function_response,
                }
            )

        second_response = client.chat.completions.create(
            model="gpt-3.5-turbo-0125",
            messages=messages,
            stream=True
        )
        return second_response

if __name__ == "__main__":
    question = "What's the weather like in Paris and San Francisco?"
    response = run_conversation(question)
    for chunk in response:
        print(chunk.choices[0].delta.content or "", end='', flush=True)
# Milestone 2