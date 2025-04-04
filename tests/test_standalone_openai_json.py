import tests.test_header
import os

# -----

import dotenv
dotenv.load_dotenv()

import json
from textwrap import dedent
from openai import OpenAI
from pydantic import BaseModel

MODEL = "gpt-4o-2024-08-06"

math_tutor_prompt = '''
    You are a helpful math tutor. You will be provided with a math problem,
    and your goal will be to output a step by step solution, along with a final answer.
    For each step, just provide the output as an equation use the explanation field to detail the reasoning.
'''

bad_prompt = '''
    Follow the instructions.
'''

client = OpenAI()

class MathReasoning(BaseModel):
    class Step(BaseModel):
        explanation: str
        output: str

    steps: list[Step]
    final_answer: str

def get_math_solution(question: str):
    completion = client.beta.chat.completions.parse(
        model=MODEL,
        messages=[
            {"role": "system", "content": dedent(bad_prompt)},
            {"role": "user", "content": question},
        ],
        response_format=MathReasoning,
    )
    return completion.choices[0].message

# Web Server
import http.server
import socketserver
import urllib.parse

PORT = 5555

class CustomHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        # Parse query parameters from the URL
        parsed_path = urllib.parse.urlparse(self.path)
        query_params = urllib.parse.parse_qs(parsed_path.query)

        # Check for a specific query parameter, e.g., 'problem'
        problem = query_params.get('problem', [''])[0]  # Default to an empty string if 'problem' isn't provided

        if problem:
            print(f"Problem: {problem}")
            solution = get_math_solution(problem)
            
            if solution.refusal:
                print(f"Refusal: {solution.refusal}")

            print(f"Solution: {solution}")
            self.send_response(200)
        else:
            solution = json.dumps({"error": "Please provide a math problem using the 'problem' query parameter."})
            self.send_response(400)

        self.send_header('Content-type', 'application/json; charset=utf-8')
        self.end_headers()

        # Write the message content
        self.wfile.write(str(solution).encode())

with socketserver.TCPServer(("", PORT), CustomHandler) as httpd:
    print(f"Serving at port {PORT}")
    httpd.serve_forever()
