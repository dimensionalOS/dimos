#!/usr/bin/env python3
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

"""Minimal TensorZero test to get it working."""

from pathlib import Path

from dotenv import load_dotenv
from openai import OpenAI
from tensorzero import patch_openai_client

load_dotenv()

# Create minimal config
config_dir = Path("/tmp/tz_test")
config_dir.mkdir(exist_ok=True)
config_path = config_dir / "tensorzero.toml"

# Minimal config based on TensorZero docs
config = """
[models.gpt_4o_mini]
routing = ["openai"]

[models.gpt_4o_mini.providers.openai]
type = "openai"
model_name = "gpt-4o-mini"

[functions.my_function]
type = "chat"

[functions.my_function.variants.my_variant]
type = "chat_completion"
model = "gpt_4o_mini"
"""

with open(config_path, "w") as f:
    f.write(config)

print(f"Created config at {config_path}")

# Create OpenAI client
client = OpenAI()

# Patch with TensorZero
try:
    patch_openai_client(
        client,
        clickhouse_url=None,  # In-memory
        config_file=str(config_path),
        async_setup=False,
    )
    print("✅ TensorZero initialized successfully!")
except Exception as e:
    print(f"❌ Failed to initialize TensorZero: {e}")
    exit(1)

# Test basic inference
print("\nTesting basic inference...")
try:
    response = client.chat.completions.create(
        model="tensorzero::function_name::my_function",
        messages=[{"role": "user", "content": "What is 2+2?"}],
        temperature=0.0,
        max_tokens=10,
    )

    content = response.choices[0].message.content
    print(f"Response: {content}")
    print("✅ Basic inference worked!")

except Exception as e:
    print(f"❌ Basic inference failed: {e}")
    import traceback

    traceback.print_exc()

print("\nTesting streaming...")
try:
    stream = client.chat.completions.create(
        model="tensorzero::function_name::my_function",
        messages=[{"role": "user", "content": "Count from 1 to 3"}],
        temperature=0.0,
        max_tokens=20,
        stream=True,
    )

    print("Stream response: ", end="", flush=True)
    for chunk in stream:
        if chunk.choices[0].delta.content:
            print(chunk.choices[0].delta.content, end="", flush=True)
    print("\n✅ Streaming worked!")

except Exception as e:
    print(f"\n❌ Streaming failed: {e}")
