from abc import ABC, abstractmethod
import tiktoken
from dimos.utils.logging_config import setup_logger

# TODO: Add a class for specific tokenizer exceptions
# TODO: Build out testing and logging
# TODO: Create proper doc strings after multiple tokenizers are implemented


class AbstractTokenizer(ABC):

    @abstractmethod
    def tokenize_text(self, text):
        pass

    @abstractmethod
    def detokenize_text(self, tokenized_text):
        pass

    @abstractmethod
    def token_count(self, text):
        pass

    @abstractmethod
    def image_token_count(self, image_width, image_height, image_detail="low"):
        pass


class OpenAI_Tokenizer(AbstractTokenizer):

    def __init__(self, model_name: str = "gpt-4o", **kwargs):
        super().__init__(**kwargs)

        # Initilize the tokenizer for the openai set of models
        self.model_name = model_name
        try:
            self.tokenizer = tiktoken.encoding_for_model(self.model_name)
        except Exception as e:
            raise ValueError(
                f"Failed to initialize tokenizer for model {self.model_name}. Error: {str(e)}"
            )

    def tokenize_text(self, text):
        """
        Tokenize a text string using the openai tokenizer.
        """
        return self.tokenizer.encode(text)

    def detokenize_text(self, tokenized_text):
        """
        Detokenize a text string using the openai tokenizer.
        """
        try:
            return self.tokenizer.decode(tokenized_text, errors="ignore")
        except Exception as e:
            raise ValueError(f"Failed to detokenize text. Error: {str(e)}")

    def token_count(self, text):
        """
        Gets the token count of a text string using the openai tokenizer.
        """
        return len(self.tokenize_text(text)) if text else 0

    @staticmethod
    def image_token_count(image_width, image_height, image_detail="high"):
        """
        Calculate the number of tokens in an image. Low detail is 85 tokens, high detail is 170 tokens per 512x512 square.
        """
        logger = setup_logger(
            "dimos.agents.tokenizer.OpenAI_Tokenizer.image_token_count")

        if image_detail == "low":
            return 85
        elif image_detail == "high":
            # Image dimensions
            logger.debug(
                f"Image Width: {image_width}, Image Height: {image_height}")
            if image_width is None or image_height is None:
                raise ValueError(
                    "Image width and height must be provided for high detail image token count calculation."
                )

            # Scale image to fit within 2048 x 2048
            max_dimension = max(image_width, image_height)
            if max_dimension > 2048:
                scale_factor = 2048 / max_dimension
                image_width = int(image_width * scale_factor)
                image_height = int(image_height * scale_factor)

            # Scale shortest side to 768px
            min_dimension = min(image_width, image_height)
            scale_factor = 768 / min_dimension
            image_width = int(image_width * scale_factor)
            image_height = int(image_height * scale_factor)

            # Calculate number of 512px squares
            num_squares = (image_width // 512) * (image_height // 512)
            return 170 * num_squares + 85
        else:
            raise ValueError(
                "Detail specification of image is not 'low' or 'high'")