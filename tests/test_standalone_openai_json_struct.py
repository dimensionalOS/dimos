import tests.test_header
import os

# -----

from typing import List, Union, Dict

import dotenv
dotenv.load_dotenv()

from textwrap import dedent
from openai import OpenAI
from pydantic import BaseModel

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

def get_math_solution(question: str):
    prompt = general_prompt
    completion = client.beta.chat.completions.parse(
        model=MODEL,
        messages=[
            {"role": "system", "content": dedent(prompt)},
            {"role": "user", "content": question},
        ],
        response_format=MathReasoning,
    )
    return completion.choices[0].message

# Define Problem
problem = "What is the derivative of 3x^2"
print(f"Problem: {problem}")

# Query for result
solution = get_math_solution(problem)

# If the query was refused
if solution.refusal:
    print(f"Refusal: {solution.refusal}")
    exit()

# If we were able to successfully parse the response back
parsed_solution = solution.parsed
if not parsed_solution:
    print(f"Unable to Parse Solution")
    exit()
    
# Print solution from class definitions
print(f"Parsed: {parsed_solution}")

steps = parsed_solution.steps
print(f"Steps: {steps}")

final_answer = parsed_solution.final_answer
print(f"Final Answer: {final_answer}")
