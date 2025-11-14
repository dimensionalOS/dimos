from transformers import SamModel, SamProcessor
import torch
import numpy as np

class SAM:
    def __init__(self, model_name="facebook/sam-vit-huge", device="cuda"):
        self.device = device
        self.sam_model = SamModel.from_pretrained(model_name).to(self.device)
        self.sam_processor = SamProcessor.from_pretrained(model_name)

    def run_inference_from_points(self, image, points):
        sam_inputs = self.sam_processor(image, input_points=points, return_tensors="pt").to(self.device)
        with torch.no_grad():
            sam_outputs = self.sam_model(**sam_inputs)
        return self.sam_processor.image_processor.post_process_masks(sam_outputs.pred_masks.cpu(), sam_inputs["original_sizes"].cpu(), sam_inputs["reshaped_input_sizes"].cpu())
