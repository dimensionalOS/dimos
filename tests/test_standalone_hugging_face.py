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

import tests.test_header

# from transformers import AutoModelForCausalLM, AutoTokenizer

# model_name = "Qwen/QwQ-32B"

# model = AutoModelForCausalLM.from_pretrained(
#     model_name,
#     torch_dtype="auto",
#     device_map="auto"
# )
# tokenizer = AutoTokenizer.from_pretrained(model_name)

# prompt = "How many r's are in the word \"strawberry\""
# messages = [
#     {"role": "user", "content": prompt}
# ]
# text = tokenizer.apply_chat_template(
#     messages,
#     tokenize=False,
#     add_generation_prompt=True
# )

# model_inputs = tokenizer([text], return_tensors="pt").to(model.device)

# generated_ids = model.generate(
#     **model_inputs,
#     max_new_tokens=32768
# )
# generated_ids = [
#     output_ids[len(input_ids):] for input_ids, output_ids in zip(model_inputs.input_ids, generated_ids)
# ]

# response = tokenizer.batch_decode(generated_ids, skip_special_tokens=True)[0]
# print(response)

# -----------------------------------------------------------------------------

# import requests
# import json

# API_URL = "https://api-inference.huggingface.co/models/Qwen/QwQ-32B"
# api_key = os.getenv('HUGGINGFACE_ACCESS_TOKEN')

# HEADERS = {"Authorization": f"Bearer {api_key}"}

# prompt = "How many r's are in the word \"strawberry\""
# messages = [
#     {"role": "user", "content": prompt}
# ]

# # Format the prompt in the desired chat format
# chat_template = (
#     f"{messages[0]['content']}\n"
#     "Assistant:"
# )

# payload = {
#     "inputs": chat_template,
#     "parameters": {
#         "max_new_tokens": 32768,
#         "temperature": 0.7
#     }
# }

# # API request
# response = requests.post(API_URL, headers=HEADERS, json=payload)

# # Handle response
# if response.status_code == 200:
#     output = response.json()[0]['generated_text']
#     print(output.strip())
# else:
#     print(f"Error {response.status_code}: {response.text}")

# -----------------------------------------------------------------------------

# import os
# import requests
# import time

# API_URL = "https://api-inference.huggingface.co/models/Qwen/QwQ-32B"
# api_key = os.getenv('HUGGINGFACE_ACCESS_TOKEN')

# HEADERS = {"Authorization": f"Bearer {api_key}"}

# def query_with_retries(payload, max_retries=5, delay=15):
#     for attempt in range(max_retries):
#         response = requests.post(API_URL, headers=HEADERS, json=payload)
#         if response.status_code == 200:
#             return response.json()[0]['generated_text']
#         elif response.status_code == 500:  # Service unavailable
#             print(f"Attempt {attempt + 1}/{max_retries}: Model busy. Retrying in {delay} seconds...")
#             time.sleep(delay)
#         else:
#             print(f"Error {response.status_code}: {response.text}")
#             break
#     return "Failed after multiple retries."

# prompt = "How many r's are in the word \"strawberry\""
# messages = [{"role": "user", "content": prompt}]
# chat_template = f"{messages[0]['content']}\nAssistant:"

# payload = {
#     "inputs": chat_template,
#     "parameters": {"max_new_tokens": 32768, "temperature": 0.7}
# }

# output = query_with_retries(payload)
# print(output.strip())

# -----------------------------------------------------------------------------

import os
from huggingface_hub import InferenceClient

# Use environment variable for API key
api_key = os.getenv('HUGGINGFACE_ACCESS_TOKEN')

client = InferenceClient(
    provider="hf-inference",
    api_key=api_key,
)

messages = [
	{
		"role": "user",
		"content": "How many r's are in the word \"strawberry\""
	}
]

completion = client.chat.completions.create(
    model="Qwen/QwQ-32B", 
	messages=messages, 
	max_tokens=150,
)

print(completion.choices[0].message)

