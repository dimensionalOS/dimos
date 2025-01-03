import base64
from openai import OpenAI
from dotenv import load_dotenv
import cv2
import reactivex as rx
from reactivex import operators as ops
from reactivex.disposable import CompositeDisposable
import os

from dotenv import load_dotenv
load_dotenv()

import threading

class Agent:
    def __init__(self, dev_name:str="NA", agent_type:str="Base"):
        self.dev_name = dev_name
        self.agent_type = agent_type
        self.disposables = CompositeDisposable()
    
    # def process_frame(self):
    #     """Processes a single frame. Should be implemented by subclasses."""
    #     raise NotImplementedError("Frame processing must be handled by subclass")

    def dispose_all(self):
        """Disposes of all active subscriptions managed by this agent."""
        if self.disposables:
            self.disposables.dispose()
        else:
            print("No disposables to dispose.")


class OpenAI_Agent(Agent):
    memory_file_lock = threading.Lock()

    def __init__(self, dev_name: str, agent_type:str="Vision", query="What do you see?", output_dir='/app/dimos/assets/agent'):
        """
        Initializes a new OpenAI_Agent instance, an agent specialized in handling vision tasks.

        Args:
            dev_name (str): The name of the device.
            agent_type (str): The type of the agent, defaulting to 'Vision'.
        """
        super().__init__(dev_name, agent_type)
        self.client = OpenAI()
        self.is_processing = False
        self.query = query
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)

    def encode_image(self, image):
        """
        Encodes an image array into a base64 string suitable for transmission.

        Args:
            image (ndarray): An image array to encode.

        Returns:
            str: The base64 encoded string of the image.
        """
        _, buffer = cv2.imencode('.jpg', image)
        if buffer is None:
            raise ValueError("Failed to encode image")
        return base64.b64encode(buffer).decode('utf-8')

    # def encode_image(self, image):
    #     """
    #     Creates an observable that encodes an image array into a base64 string.

    #     Args:
    #         image (ndarray): An image array to encode.

    #     Returns:
    #         Observable: An observable that emits the base64 encoded string of the image.
    #     """
    #     def observable_image_encoder(observer, scheduler):
    #         try:
    #             _, buffer = cv2.imencode('.jpg', image)
    #             if buffer is None:
    #                 observer.on_error(ValueError("Failed to encode image"))
    #             else:
    #                 encoded_string = base64.b64encode(buffer).decode('utf-8')
    #                 observer.on_next(encoded_string)
    #                 observer.on_completed()
    #         except Exception as e:
    #             observer.on_error(e)

    #     return rx.create(observable_image_encoder)

    def query_openai_with_image(self, base64_image):
        """
        Sends an encoded image to OpenAI's API for analysis and returns the response.

        Args:
            base64_image (str): The base64 encoded string of the image.
            query (str): The query text to accompany the image.

        Returns:
            str: The content of the response from OpenAI.
        """
        try:
            response = self.client.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {"role": "user", "content": [{"type": "text", "text": self.query},
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_image}", "detail": "high"}}]},
                ],
                max_tokens=300,
            )
            return response.choices[0].message.content
        except Exception as e:
            print(f"API request failed: {e}")
            return None

    # def query_openai_with_image(self, base64_image, query="What’s in this image?"):
    #     """
    #     Creates an observable that sends an encoded image to OpenAI's API for analysis.

    #     Args:
    #         base64_image (str): The base64 encoded string of the image.
    #         query (str): The query text to accompany the image.

    #     Returns:
    #         Observable: An observable that emits the response from OpenAI.
    #     """
    #     def observable_openai_query(observer, scheduler):
    #         try:
    #             response = self.client.chat.completions.create(
    #                 model="gpt-4o",
    #                 messages=[
    #                     {"role": "user", "content": [{"type": "text", "text": query},
    #                     {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_image}", "detail": "high"}}]},
    #                 ],
    #                 max_tokens=300,
    #             )
    #             if response:
    #                 observer.on_next(response.choices[0].message.content)
    #                 observer.on_completed()
    #             else:
    #                 observer.on_error(Exception("Failed to get a valid response from OpenAI"))
    #         except Exception as e:
    #             print(f"API request failed: {e}")
    #             observer.on_error(e)

    #     return rx.create(observable_openai_query)

    # def send_query_and_handle_timeout(self, image_base64):
    #     """
    #     Sends an image query to OpenAI and handles response or timeout.

    #     Args:
    #         image_base64 (str): Base64 encoded string of the image to query.

    #     Returns:
    #         Observable: Observable emitting either OpenAI response or timeout signal.
    #     """
    #     # Setting a timeout for the OpenAI request
    #     timeout_seconds = 10  # Timeout after 10 seconds
    #     return rx.of(image_base64).pipe(
    #         ops.map(self.query_openai_with_image),
    #         ops.timeout(timeout_seconds),
    #         ops.catch(rx.catch(handler=lambda e: rx.of(f"Timeout or error occurred: {e}")))
    #     )
    
    # def process_image_stream(self, image_stream):
    #     """
    #     Processes an image stream by encoding images and querying OpenAI.

    #     Args:
    #         image_stream (Observable): An observable stream of image arrays.

    #     Returns:
    #         Observable: An observable stream of OpenAI responses.
    #     """
    #     return image_stream.pipe(
    #         ops.map(self.encode_image),  # Assume this returns a base64 string immediately
    #         ops.exhaust_map(lambda image_base64: self.send_query_and_handle_timeout(image_base64))
    #     )

    def process_if_idle(self, image):
        if not self.is_processing:
            self.is_processing = True  # Set processing flag
            return self.encode_image(image).pipe(
                ops.flat_map(self.query_openai_with_image),
                ops.do_action(on_next=lambda _: None, on_completed=lambda: self.reset_processing_flag())
            )
        else:
            return rx.empty()  # Ignore the emission if already processing

    def reset_processing_flag(self):
        self.is_processing = False

    def process_image_stream(self, image_stream):
        """
        Processes an image stream by encoding images and querying OpenAI.

        Args:
            image_stream (Observable): An observable stream of image arrays.

        Returns:
            Observable: An observable stream of OpenAI responses.
        """
        # Process each and every entry, one after another
        return image_stream.pipe(
            ops.map(self.encode_image),
            ops.map(self.query_openai_with_image),
        )
        
        # Process image, ignoring new images while processing
        # return image_stream.pipe(
        #     ops.flat_map(self.process_if_idle),
        #     ops.filter(lambda x: x is not None)  # Filter out ignored (None) emissions
        # )
    
    def subscribe_to_image_processing(self, frame_observable):
        """
        Subscribes to an observable of frames, processes them, and handles the responses.

        Args:
            frame_observable (Observable): An observable stream of image frames.
        """
        disposable = self.process_image_stream(frame_observable).subscribe(
            on_next=self.log_response_to_file, # lambda response: print(f"OpenAI Response [{self.dev_name}]:", response),
            on_error=lambda e: print("Error:", e),
            on_completed=lambda: print("Stream processing completed.")
        )
        self.disposables.add(disposable)
    
    def log_response_to_file(self, response):
        """
        Logs the response to a shared 'memory.txt' file with the device name prefixed,
        using a lock to ensure thread safety.

        Args:
            response (str): The response to log.
        """
        with open('/app/dimos/assets/agent/memory.txt', 'a') as file:
            file.write(f"{self.dev_name}: {response}\n")
            print(f"OpenAI Response [{self.dev_name}]:", response)