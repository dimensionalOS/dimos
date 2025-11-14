from transformers import AutoProcessor, CLIPSegForImageSegmentation
import torch
import numpy as np

class CLIPSeg:
    def __init__(self, model_name="CIDAS/clipseg-rd64-refined"):
        self.clipseg_processor = AutoProcessor.from_pretrained(model_name)
        self.clipseg_model = CLIPSegForImageSegmentation.from_pretrained(model_name)

    def run_inference(self, image, text_descriptions):
        inputs = self.clipseg_processor(text=text_descriptions, images=[image] * len(text_descriptions), padding=True, return_tensors="pt")
        outputs = self.clipseg_model(**inputs)
        logits = outputs.logits
        return logits.detach().unsqueeze(1)