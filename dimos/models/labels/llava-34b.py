import json
import os

# llava v1.6
from llama_cpp import Llama
from llama_cpp.llama_chat_format import Llava15ChatHandler

from vqasynth.datasets.utils import image_to_base64_data_uri

class Llava:
    def __init__(self, mmproj=f"{os.getcwd()}/models/mmproj-model-f16.gguf", model_path=f"{os.getcwd()}/models/llava-v1.6-34b.Q4_K_M.gguf", gpu=True):
        chat_handler = Llava15ChatHandler(clip_model_path=mmproj, verbose=True)
        n_gpu_layers = 0
        if gpu:
           n_gpu_layers = -1
        self.llm = Llama(model_path=model_path, chat_handler=chat_handler, n_ctx=2048, logits_all=True, n_gpu_layers=n_gpu_layers)

    def run_inference(self, image, prompt, return_json=True):
        data_uri = image_to_base64_data_uri(image)
        res = self.llm.create_chat_completion(
             messages = [
                 {"role": "system", "content": "You are an assistant who perfectly describes images."},
                 {
                     "role": "user",
                     "content": [
                         {"type": "image_url", "image_url": {"url": data_uri}},
                         {"type" : "text", "text": prompt}
                    ]
                 }
               ]
             )
        if return_json:

            return list(set(self.extract_descriptions_from_incomplete_json(res["choices"][0]["message"]["content"])))

        return res["choices"][0]["message"]["content"]

    def extract_descriptions_from_incomplete_json(self, json_like_str):
        last_object_idx = json_like_str.rfind(',"object')

        if last_object_idx != -1:
            json_str = json_like_str[:last_object_idx] + '}'
        else:
            json_str = json_like_str.strip()
            if not json_str.endswith('}'):
                json_str += '}'

        try:
            json_obj = json.loads(json_str)
            descriptions = [details['description'].replace(".","") for key, details in json_obj.items() if 'description' in details]

            return descriptions
        except json.JSONDecodeError as e:
            raise ValueError(f"Error parsing JSON: {e}")
