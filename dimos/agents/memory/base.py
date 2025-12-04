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

from abc import abstractmethod

from dimos.exceptions.agent_memory_exceptions import (
    AgentMemoryConnectionError,
    UnknownConnectionTypeError,
)
from dimos.utils.logging_config import setup_logger

# TODO
# class AbstractAgentMemory(ABC):

# TODO
# class AbstractAgentSymbolicMemory(AbstractAgentMemory):


class AbstractAgentSemanticMemory:  # AbstractAgentMemory):
    def __init__(self, connection_type: str = "local", **kwargs) -> None:  # type: ignore[no-untyped-def]
        """
        Initialize with dynamic connection parameters.
        Args:
            connection_type (str): 'local' for a local database, 'remote' for a remote connection.
        Raises:
            UnknownConnectionTypeError: If an unrecognized connection type is specified.
            AgentMemoryConnectionError: If initializing the database connection fails.
        """
        self.logger = setup_logger()
        self.logger.info("Initializing AgentMemory with connection type: %s", connection_type)
        self.connection_params = kwargs
        self.db_connection = (
            None  # Holds the conection, whether local or remote, to the database used.
        )

        if connection_type not in ["local", "remote"]:
            error = UnknownConnectionTypeError(
                f"Invalid connection_type {connection_type}. Expected 'local' or 'remote'."
            )
            self.logger.error(str(error))
            raise error

        try:
            if connection_type == "remote":
                self.connect()  # type: ignore[no-untyped-call]
            elif connection_type == "local":
                self.create()  # type: ignore[no-untyped-call]
        except Exception as e:
            self.logger.error("Failed to initialize database connection: %s", str(e), exc_info=True)
            raise AgentMemoryConnectionError(
                "Initialization failed due to an unexpected error.", cause=e
            ) from e

    @abstractmethod
    def connect(self):  # type: ignore[no-untyped-def]
        """Establish a connection to the data store using dynamic parameters specified during initialization."""

    @abstractmethod
    def create(self):  # type: ignore[no-untyped-def]
        """Create a local instance of the data store tailored to specific requirements."""

    ## Create ##
    @abstractmethod
    def add_vector(self, vector_id, vector_data):  # type: ignore[no-untyped-def]
        """Add a vector to the database.
        Args:
            vector_id (any): Unique identifier for the vector.
            vector_data (any): The actual data of the vector to be stored.
        """

    ## Read ##
    @abstractmethod
    def get_vector(self, vector_id):  # type: ignore[no-untyped-def]
        """Retrieve a vector from the database by its identifier.
        Args:
            vector_id (any): The identifier of the vector to retrieve.
        """

    @abstractmethod
    def query(self, query_texts, n_results: int = 4, similarity_threshold=None):  # type: ignore[no-untyped-def]
        """Performs a semantic search in the vector database.

        Args:
            query_texts (Union[str, List[str]]): The query text or list of query texts to search for.
            n_results (int, optional): Number of results to return. Defaults to 4.
            similarity_threshold (float, optional): Minimum similarity score for results to be included [0.0, 1.0]. Defaults to None.

        Returns:
            List[Tuple[Document, Optional[float]]]: A list of tuples containing the search results. Each tuple
            contains:
                Document: The retrieved document object.
                Optional[float]: The similarity score of the match, or None if not applicable.

        Raises:
            ValueError: If query_texts is empty or invalid.
            ConnectionError: If database connection fails during query.
        """

    ## Update ##
    @abstractmethod
    def update_vector(self, vector_id, new_vector_data):  # type: ignore[no-untyped-def]
        """Update an existing vector in the database.
        Args:
            vector_id (any): The identifier of the vector to update.
            new_vector_data (any): The new data to replace the existing vector data.
        """

    ## Delete ##
    @abstractmethod
    def delete_vector(self, vector_id):  # type: ignore[no-untyped-def]
        """Delete a vector from the database using its identifier.
        Args:
            vector_id (any): The identifier of the vector to delete.
        """


# query(string, metadata/tag, n_rets, kwargs)

# query by string, timestamp, id, n_rets

# (some sort of tag/metadata)

# temporal
