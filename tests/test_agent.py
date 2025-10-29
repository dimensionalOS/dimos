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

import os

# -----
from dotenv import load_dotenv


# Sanity check for dotenv
def test_dotenv():
    print("test_dotenv:")
    load_dotenv()
    openai_api_key = os.getenv("OPENAI_API_KEY")
    print("\t\tOPENAI_API_KEY: ", openai_api_key)


# Sanity check for openai connection
def test_openai_connection():
    from openai import OpenAI

    client = OpenAI()
    print("test_openai_connection:")
    response = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": "What's in this image?"},
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": "https://upload.wikimedia.org/wikipedia/commons/thumb/d/dd/Gfp-wisconsin-madison-the-nature-boardwalk.jpg/2560px-Gfp-wisconsin-madison-the-nature-boardwalk.jpg",
                        },
                    },
                ],
            }
        ],
        max_tokens=300,
    )
    print("\t\tOpenAI Response: ", response.choices[0])


test_dotenv()
test_openai_connection()
