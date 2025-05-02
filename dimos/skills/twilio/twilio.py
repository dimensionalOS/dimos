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
from typing import Optional, Dict, Any, List, Tuple
from pydantic import Field
import logging
from twilio.rest import Client
from twilio.base.exceptions import TwilioRestException
import os
from dotenv import load_dotenv
logger = logging.getLogger(__name__)

# Twilio Env Vars
load_dotenv()

class SendTwilioSmsSkill(AbstractSkill):
    """Sends an SMS message using Twilio.

    Requires Twilio Account SID, Auth Token, a recipient number ('to'),
    a Twilio sending number ('from'), and the message body.

    Attributes:
        to_number: The recipient's phone number in E.164 format (e.g., +14155552671).
        body: The text content of the SMS message. Max 1600 characters.
    """

    to_number: str = Field(..., description="Recipient phone number (E.164 format).")
    body: str = Field(..., max_length=1600, description="SMS message body (max 1600 chars).")

    def __call__(self) -> str:
        """Executes the Twilio API call to send the SMS.

        Returns:
            A string indicating success (including the message SID) or failure
            with an error message from the Twilio API or other exception.
        """
        # Setup Twilio client
        self._from_number = os.getenv("TWILIO_PHONE_NUMBER")
        self._account_sid = os.getenv("TWILIO_ACCOUNT_SID")
        self._auth_token = os.getenv("TWILIO_AUTH_TOKEN")

        logger.info(f"Attempting to send SMS via Twilio from {self._from_number} to {self.to_number}")
        try:
            # TODO: Consider initializing the client once if the skill instance is reused
            client = Client(self._account_sid, self._auth_token)
            message = client.messages.create(
                to=self.to_number,
                from_=self._from_number,
                body=self.body
            )

            success_msg = f"SMS sent successfully. SID: {message.sid}, Status: {message.status}"
            logger.info(success_msg)
            return success_msg

        except TwilioRestException as e:
            error_msg = f"Twilio API error: {e}"
            logger.error(error_msg)
            return error_msg
        except Exception as e:
            error_msg = f"An unexpected error occurred sending SMS: {type(e).__name__}: {e}"
            logger.exception(error_msg)
            return error_msg