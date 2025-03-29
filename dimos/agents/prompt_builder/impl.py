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


from textwrap import dedent
from dimos.agents.tokenizer.openai_impl import AbstractTokenizer, OpenAI_Tokenizer

# TODO: Make class more generic when implementing other tokenizers. Presently its OpenAI specific.
# TODO: Build out testing and logging

class PromptBuilder():

    DEFAULT_SYSTEM_PROMPT = dedent("""
    You are an AI assistant capable of understanding and analyzing both visual and textual information. 
    Your task is to provide accurate and insightful responses based on the data provided to you. 
    Use the following information to assist the user with their query. Do not rely on any internal 
    knowledge or make assumptions beyond the provided data.

    Visual Context: You may have been given an image to analyze. Use the visual details to enhance your response.
    Textual Context: There may be some text retrieved from a relevant database to assist you

    Instructions:
    - Combine insights from both the image and the text to answer the user's question.
    - If the information is insufficient to provide a complete answer, acknowledge the limitation.
    - Maintain a professional and informative tone in your response.
    """)
    
    def __init__(self, model_name='gpt-4o', max_tokens=128000, tokenizer: AbstractTokenizer = None):
        """
        Initialize the prompt builder.
        Args:
            model_name (str): Model used (e.g., 'gpt-4o', 'gpt-4', 'gpt-3.5-turbo').
            max_tokens (int): Maximum tokens allowed in the input prompt.
            tokenizer (AbstractTokenizer): The tokenizer to use for token counting and truncation.
        """
        self.model_name = model_name
        self.max_tokens = max_tokens
        self.tokenizer: AbstractTokenizer = tokenizer or OpenAI_Tokenizer(model_name=self.model_name)
    
    def truncate_tokens(self, text, max_tokens, strategy):
        """
        Truncate text to fit within max_tokens using a specified strategy.
        Args:
            text (str): Input text to truncate.
            max_tokens (int): Maximum tokens allowed.
            strategy (str): Truncation strategy ('truncate_head', 'truncate_middle', 'truncate_end', 'do_not_truncate').
        Returns:
            str: Truncated text.
        """
        if strategy == "do_not_truncate" or not text:
            return text

        tokens = self.tokenizer.tokenize_text(text)
        if len(tokens) <= max_tokens:
            return text

        if strategy == "truncate_head":
            truncated = tokens[-max_tokens:]
        elif strategy == "truncate_end":
            truncated = tokens[:max_tokens]
        elif strategy == "truncate_middle":
            half = max_tokens // 2
            truncated = tokens[:half] + tokens[-half:]
        else:
            raise ValueError(f"Unknown truncation strategy: {strategy}")

        return self.tokenizer.detokenize_text(truncated)
    
    def build(
        self,
        system_prompt=None,
        user_query=None,
        base64_image=None,
        image_width=None,
        image_height=None,
        image_detail="low",
        rag_context=None,
        budgets=None,
        policies=None,
        override_token_limit=False,
    ):
        """
        Builds a dynamic prompt tailored to token limits, respecting budgets and policies.

        Args:
            system_prompt (str): System-level instructions.
            user_query (str, optional): User's query.
            base64_image (str, optional): Base64-encoded image string.
            image_width (int, optional): Width of the image.
            image_height (int, optional): Height of the image.
            image_detail (str, optional): Detail level for the image ("low" or "high").
            rag_context (str, optional): Retrieved context.
            budgets (dict, optional): Token budgets for each input type. Defaults to equal allocation.
            policies (dict, optional): Truncation policies for each input type.
            override_token_limit (bool, optional): Whether to override the token limit. Defaults to False.

        Returns:
            dict: Messages array ready to send to the OpenAI API.
        """
        if user_query is None:
            raise ValueError("User query is required.")

        # Debug:
        # base64_image = None

        budgets = budgets or {
            "system_prompt": self.max_tokens // 4,
            "user_query": self.max_tokens // 4,
            "image": self.max_tokens // 4,
            "rag": self.max_tokens // 4,
        }
        policies = policies or {
            "system_prompt": "truncate_end",
            "user_query": "truncate_middle",
            "image": "do_not_truncate",
            "rag": "truncate_end",
        }

        # Validate and sanitize image_detail
        if image_detail not in {"low", "high"}:
            image_detail = "low"  # Default to "low" if invalid or None

        # Determine which system prompt to use
        if system_prompt is None:
            system_prompt = self.DEFAULT_SYSTEM_PROMPT

        rag_context = rag_context or ""
        
        # Debug:
        # print("system_prompt: ", system_prompt)
        # print("rag_context: ", rag_context)

        # region Token Counts
        if not override_token_limit:
            rag_token_cnt = self.tokenizer.token_count(rag_context)
            system_prompt_token_cnt = self.tokenizer.token_count(system_prompt)
            user_query_token_cnt = self.tokenizer.token_count(user_query)
            image_token_cnt = self.tokenizer.image_token_count(image_width, image_height, image_detail) if base64_image else 0
        else:
            rag_token_cnt = 0
            system_prompt_token_cnt = 0
            user_query_token_cnt = 0
            image_token_cnt = 0
        # endregion Token Counts

        # Create a component dictionary for dynamic allocation
        components = {
            "system_prompt": {"text": system_prompt, "tokens": system_prompt_token_cnt},
            "user_query": {"text": user_query, "tokens": user_query_token_cnt},
            "image": {"text": None, "tokens": image_token_cnt},
            "rag": {"text": rag_context, "tokens": rag_token_cnt},
        }

        if not override_token_limit:
            # Adjust budgets and apply truncation
            total_tokens = sum(comp["tokens"] for comp in components.values())
            excess_tokens = total_tokens - self.max_tokens
            if excess_tokens > 0:
                for key, component in components.items():
                    if excess_tokens <= 0:
                        break
                    if policies[key] != "do_not_truncate":
                        max_allowed = max(0, budgets[key] - excess_tokens)
                        components[key]["text"] = self.truncate_tokens(
                            component["text"], max_allowed, policies[key]
                        )
                        tokens_after = self.tokenizer.token_count(components[key]["text"])
                        excess_tokens -= component["tokens"] - tokens_after
                        component["tokens"] = tokens_after

        # Build the `messages` structure (OpenAI specific)
        messages = [{"role": "system", "content": components["system_prompt"]["text"]}]

        if components["rag"]["text"]:
            user_content = [{"type": "text", "text": f"{components['rag']['text']}\n\n{components['user_query']['text']}"}]
        else:
            user_content = [{"type": "text", "text": components["user_query"]["text"]}]

        if base64_image:
            user_content.append({
                "type": "image_url",
                "image_url": {
                    "url": f"data:image/jpeg;base64,{base64_image}",
                    "detail": image_detail,
                },
            })
        messages.append({"role": "user", "content": user_content})

        # Debug:
        # print("system_prompt: ", system_prompt)
        # print("user_query: ", user_query)
        # print("user_content: ", user_content)
        # print(f"Messages: {messages}")

        return messages
