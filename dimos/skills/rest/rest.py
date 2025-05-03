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

import requests
from dimos.skills.skills import AbstractSkill
from typing import Optional, Dict, Any
from pydantic import Field
import logging
logger = logging.getLogger(__name__)

class GenericRestSkill(AbstractSkill):
    """Performs a configurable REST API call.

    This skill executes an HTTP request based on the provided parameters. It
    supports various HTTP methods and allows specifying URL, timeout.

    Attributes:
        url: The target URL for the API call.
        method: The HTTP method (e.g., 'GET', 'POST'). Case-insensitive.
        timeout: Request timeout in seconds.
    """ 
    # TODO: Add query parameters, request body data (form-encoded or JSON), and headers.
    #, query
    # parameters, request body data (form-encoded or JSON), and headers.
    # params: Optional dictionary of URL query parameters.
    # data: Optional dictionary for form-encoded request body data.
    # json_payload: Optional dictionary for JSON request body data. Use the
    #     alias 'json' when initializing.
    # headers: Optional dictionary of HTTP headers.
    url: str = Field(..., description="The target URL for the API call.")
    method: str = Field(..., description="HTTP method (e.g., 'GET', 'POST').")
    timeout: int = Field(..., description="Request timeout in seconds.")
    # params: Optional[Dict[str, Any]] = Field(default=None, description="URL query parameters.")
    # data: Optional[Dict[str, Any]] = Field(default=None, description="Form-encoded request body.")
    # json_payload: Optional[Dict[str, Any]] = Field(default=None, alias="json", description="JSON request body.")
    # headers: Optional[Dict[str, str]] = Field(default=None, description="HTTP headers.")


    def __call__(self) -> str:
        """Executes the configured REST API call.

        Returns:
            The text content of the response on success (HTTP 2xx).

        Raises:
            requests.exceptions.RequestException: If a connection error, timeout,
                or other request-related issue occurs.
            requests.exceptions.HTTPError: If the server returns an HTTP 4xx or
                5xx status code.
            Exception: For any other unexpected errors during execution.

        Returns:
             A string representing the success or failure outcome. If successful,
             returns the response body text. If an error occurs, returns a
             descriptive error message.
        """
        try:
            logger.debug(
                f"Executing {self.method.upper()} request to {self.url} "
                f"with timeout={self.timeout}" # , params={self.params}, "
                # f"data={self.data}, json={self.json_payload}, headers={self.headers}"
            )
            response = requests.request(
                method=self.method.upper(), # Normalize method to uppercase
                url=self.url,
                # params=self.params,
                # data=self.data,
                # json=self.json_payload, # Use the attribute name defined in Pydantic
                # headers=self.headers,
                timeout=self.timeout
            )
            response.raise_for_status()  # Raises HTTPError for bad responses (4xx or 5xx)
            logger.debug(f"Request successful. Status: {response.status_code}, Response: {response.text[:100]}...")
            return response.text # Return text content directly
        except requests.exceptions.HTTPError as http_err:
            logger.error(f"HTTP error occurred: {http_err} - Status Code: {http_err.response.status_code}")
            return f"HTTP error making {self.method.upper()} request to {self.url}: {http_err.response.status_code} {http_err.response.reason}"
        except requests.exceptions.RequestException as req_err:
            logger.error(f"Request exception occurred: {req_err}")
            return f"Error making {self.method.upper()} request to {self.url}: {req_err}"
        except Exception as e:
            logger.exception(f"An unexpected error occurred: {e}") # Log the full traceback
            return f"An unexpected error occurred: {type(e).__name__}: {e}"
