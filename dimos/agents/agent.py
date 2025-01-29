import base64
import os
import threading
from typing import Tuple
import cv2
from dotenv import load_dotenv
from openai import NOT_GIVEN, OpenAI
from reactivex import create, empty, Observable, concat
from reactivex import operators as ops
from reactivex.disposable import CompositeDisposable, Disposable
import multiprocessing
from reactivex.scheduler import ThreadPoolScheduler

from dimos.agents.memory.base import AbstractAgentSemanticMemory
from dimos.agents.memory.chroma_impl import AgentSemanticMemory
from dimos.agents.prompt_builder.impl import PromptBuilder
from dimos.agents.tokenizer.openai_impl import AbstractTokenizer, OpenAI_Tokenizer

# Initialize environment variables
load_dotenv()

# Scheduler for thread pool
optimal_thread_count = multiprocessing.cpu_count()
pool_scheduler = ThreadPoolScheduler(optimal_thread_count)

# region Agent
class Agent:
    def __init__(self, dev_name: str = "NA", agent_type: str = "Base", agent_memory: AbstractAgentSemanticMemory = None):
        """
        Initializes a new instance of the Agent.

        Args:
            dev_name (str): The device name of the agent.
            agent_type (str): The type of the agent (e.g., 'Base', 'Vision'). Currently unused.
            agent_memory (AbstractAgentSemanticMemory): The memory system for the agent.
        """
        self.dev_name = dev_name
        self.agent_type = agent_type
        self.agent_memory = agent_memory or AgentSemanticMemory()
        self.disposables = CompositeDisposable()

    def dispose_all(self):
        """
        Disposes of all active subscriptions managed by this agent.
        """
        if self.disposables:
            self.disposables.dispose()
        else:
            print("No disposables to dispose.")

# endregion Agent

# region OpenAI Agent
class OpenAI_Agent(Agent):
    memory_file_lock = threading.Lock()

    def __init__(self, 
                 dev_name: str, 
                 agent_type: str = "Vision",
                 query: str = "What do you see?", 
                 output_dir: str = '/app/assets/agent', 
                 agent_memory: AbstractAgentSemanticMemory = None,
                 system_query: str = None, 
                 system_query_without_documents: str = None,
                 max_input_tokens_per_request: int = 128000,
                 max_output_tokens_per_request: int = 16384,
                 model_name: str = "gpt-4o",
                 prompt_builder: PromptBuilder = None,
                 tokenizer: AbstractTokenizer = OpenAI_Tokenizer(),
                 rag_query_n: int = 4,
                 rag_similarity_threshold: float = 0.45,
                 json_mode: bool = True):  # TODO
        """
        Initializes a new instance of the OpenAI_Agent.

        Args:
            dev_name (str): The device name of the agent.
            agent_type (str): The type of the agent, defaulting to 'Vision'. Currently unused.
            query (str): The default query to send along with images to OpenAI.
            output_dir (str): Directory where output files are stored.
            agent_memory (AbstractAgentSemanticMemory): The memory system for the agent.
            system_query (str): The system query template when documents are available.
            system_query_without_documents (str): The system query template when no documents are available.
            max_input_tokens_per_request (int): The maximum number of input tokens allowed per request.
            max_output_tokens_per_request (int): The maximum number of output tokens allowed per request.
            model_name (str): The model name to be used for the prompt builder.
            prompt_builder (PromptBuilder): An instance of the PromptBuilder to create prompts.
            tokenizer (AbstractTokenizer): The tokenizer to use for tokenization.
            rag_query_n (int): The number of results to retrieve for the RAG query.
            rag_similarity_threshold (float): The similarity threshold for RAG queries.
            json_mode (bool): Whether to use JSON mode. Defaults to True.
        """
        super().__init__(dev_name, agent_type, agent_memory or AgentSemanticMemory())
        self.client = OpenAI()
        self.query = query
        self.output_dir = output_dir
        self.system_query = system_query
        self.system_query_without_documents = system_query_without_documents
        os.makedirs(self.output_dir, exist_ok=True)
        self.is_processing = threading.Event()

        # JSON Mode
        self.json_mode = json_mode  # TODO

        # Prompt Builder
        self.model_name = model_name
        self.prompt_builder = prompt_builder or PromptBuilder(self.model_name)

        # Tokenizer
        self.tokenizer: AbstractTokenizer = tokenizer or OpenAI_Tokenizer(model_name=self.model_name)

        # Depth of RAG Query
        self.rag_query_n = rag_query_n
        self.rag_similarity_threshold = rag_similarity_threshold

        # Allocated tokens to each api call of this agent.
        self.max_output_tokens_per_request = max_output_tokens_per_request
        self.max_input_tokens_per_request = max_input_tokens_per_request
        self.max_tokens_per_request = max_input_tokens_per_request + max_output_tokens_per_request     

        # Add to agent memory (TODO: Remove/Restructure)
        # Context should be able to be added, but should not be placed here statically as such.
        self.agent_memory.add_vector("id0", "Optical Flow is a technique used to track the movement of objects in a video sequence.")
        self.agent_memory.add_vector("id1", "Edge Detection is a technique used to identify the boundaries of objects in an image.")
        self.agent_memory.add_vector("id2", "Video is a sequence of frames captured at regular intervals.")
        self.agent_memory.add_vector("id3", "Colors in Optical Flow are determined by the movement of light, and can be used to track the movement of objects.")
        self.agent_memory.add_vector("id4", "Json is a data interchange format that is easy for humans to read and write, and easy for machines to parse and generate.")

    # Milestone 9

    def _observable_query(self, observer, base64_image, dimensions, override_token_limit=False, image_detail="low"):
        """
        Helper method to query OpenAI with an optional encoded image and emit to an observer.

        Args:
            observer (Observer): The observer to emit to.
            base64_image (str): The Base64-encoded image to send.

        Raises:
            Exception: If the query to OpenAI fails.
        """
        try:
            # region RAG Context
            # Get the RAG Context
            results = self.agent_memory.query(
                query_texts=self.query,
                n_results=self.rag_query_n,
                similarity_threshold=self.rag_similarity_threshold
            )

            # Pretty format the query results
            formatted_results = "\n".join(
                f"Document ID: {doc.id}\nMetadata: {doc.metadata}\nContent: {doc.page_content}\nScore: {score}\n"
                for (doc, score) in results
            )
            print(f"Agent Memory Query Results:\n{formatted_results}")
            print(f"=== Results End ===")

            # Condensed results as a single string
            condensed_results = " | ".join(
                f"{doc.page_content}"
                for (doc, _) in results
            )
            # endregion RAG Context

            # region Prompt Builder
            # Define budgets and policies
            budgets = {
                "system_prompt": self.max_input_tokens_per_request // 4,
                "user_query": self.max_input_tokens_per_request // 4,
                "image": self.max_input_tokens_per_request // 4,
                "rag": self.max_input_tokens_per_request // 4,
            }
            policies = {
                "system_prompt": "truncate_end",
                "user_query": "truncate_middle",
                "image": "do_not_truncate",
                "rag": "truncate_end",
            }

            # Prompt Builder
            messages = self.prompt_builder.build(
                user_query=self.query, 
                override_token_limit=override_token_limit,
                base64_image=base64_image, 
                image_width=dimensions[0],
                image_height=dimensions[1],
                image_detail=image_detail,
                rag_context=condensed_results, 
                fallback_system_prompt=self.system_query_without_documents,
                system_prompt=self.system_query,
                budgets=budgets,
                policies=policies,
                json_mode=self.json_mode  # TODO
            )
            # endregion Dynamic Prompt Builder
            
            # region OpenAI API Call
            # TODO: JSON Mode
            response = self.client.chat.completions.create(
                model=self.model_name,
                messages=messages,
                max_tokens=self.max_output_tokens_per_request,
                response_format={"type": "json_object"}
            )
            observer.on_next(response.choices[0].message.content)
            observer.on_completed()
            # endregion OpenAI API Call
        
        except Exception as e:
            print(f"[ERROR] OpenAI query failed in {self.dev_name}: {e}")
            observer.on_error(e)

    # Milestone 9

    # region Image Encoding / Decoding / Processing
    def _encode_image(self, image):
        """
        Encodes an image to JPEG format and converts it to a Base64 string.

        Args:
            image (numpy.ndarray): The image to encode.

        Returns:
            Observable: An Observable that emits the Base64 string of the encoded image.
        """
        return create(lambda observer, _: self._observable_encode(observer, image))

    def _observable_encode(self, observer, image):
        """
        Helper method to encode image and emit to an observer.

        Args:
            observer (Observer): The observer to emit to.
            image (numpy.ndarray): The image to encode.

        Raises:
            ValueError: If the image cannot be encoded.
        """
        try:
            width, height = image.shape[:2]
            _, buffer = cv2.imencode('.jpg', image)
            if buffer is None:
                observer.on_error(ValueError("Failed to encode image"))
            base64_image = base64.b64encode(buffer).decode('utf-8')
            observer.on_next((base64_image, (width, height)))
            observer.on_completed()
        except Exception as e:
            observer.on_error(e)

    def _query_openai_with_image(self, base64_image: str, dimensions: Tuple[int, int]) -> Observable:
        """
        Sends an encoded image to OpenAI and gets a response.

        Args:
            base64_image (str): The Base64-encoded image to send.
            dimensions (Tuple[int, int]): A tuple containing the width and height of the image.

        Returns:
            Observable: An Observable that emits the response from OpenAI.
        """
        return create(lambda observer, _: self._observable_query(observer, base64_image, dimensions))


    def _process_if_idle(self, image):
        """
        Processes an image if the agent is idle.

        Args:
            image (numpy.ndarray): The image to process.

        Returns:
            Observable: An Observable that handles the processing of the image.
        """
        if not self.is_processing.is_set():
            print("Processing Frame.")
            self.is_processing.set()
            return self._encode_image(image).pipe(
                ops.flat_map(lambda base64_and_dims: self._query_openai_with_image(*base64_and_dims)),
                ops.do_action(on_next=lambda _: None, on_completed=self._reset_processing_flag)
            )
        else:
            print("Skipping Frame.")
            return empty()
    
    def _reset_processing_flag(self):
        """
        Resets the processing flag to allow new image processing.
        """
        self.is_processing.clear()

    def _process_image_stream(self, image_stream):
        """
        Processes a stream of images.

        Args:
            image_stream (Observable): The stream of images to process.

        Returns:
            Observable: An Observable that processes each image in the stream.
        """
        return image_stream.pipe(
            ops.observe_on(pool_scheduler),
            ops.flat_map(self._process_if_idle),
            ops.filter(lambda x: x is not None)
        )

    def subscribe_to_image_processing(
        self, 
        frame_observable: Observable
    ) -> Disposable:
        """Subscribes to and processes a stream of video frames.

        Sets up a subscription to process incoming video frames through OpenAI's
        vision model. Each frame is processed only when the agent is idle to prevent
        overwhelming the API. Responses are logged to a file and the subscription
        is tracked for cleanup.

        Args:
            frame_observable: An Observable emitting video frames.
                Each frame should be a numpy array in BGR format with shape
                (height, width, 3).

        Returns:
            A Disposable representing the subscription. Can be used for external
            resource management while still being tracked internally.

        Raises:
            TypeError: If frame_observable is not an Observable.
            ValueError: If frames have invalid format or dimensions.

        Example:
            >>> agent = OpenAI_Agent("camera_1")
            >>> disposable = agent.subscribe_to_image_processing(frame_stream)
            >>> # Later cleanup
            >>> disposable.dispose()

        Note:
            The subscription is automatically added to the agent's internal
            CompositeDisposable for cleanup. The returned Disposable provides
            additional control if needed.
        """
        disposable = self._process_image_stream(frame_observable).subscribe(
            on_next=self._log_response_to_file,
            on_error=lambda e: print(f"Error in {self.dev_name}: {e}"),
            on_completed=lambda: print(f"Stream processing completed for {self.dev_name}")
        )
        self.disposables.add(disposable)
        return disposable
    
    # endregion Image Encoding / Decoding / Processing

    # region Logging
    def _log_response_to_file(self, response):
        """
        Logs the response from OpenAI to a file.

        Args:
            response (str): The response from OpenAI to log.
        """
        with self.memory_file_lock:
            with open(os.path.join(self.output_dir, 'memory.txt'), 'a') as file:
                file.write(f"{self.dev_name}: {response}\n")
                print(f"[INFO] OpenAI Response [{self.dev_name}]: {response}")
    # endregion Logging

# endregion OpenAI Agent
