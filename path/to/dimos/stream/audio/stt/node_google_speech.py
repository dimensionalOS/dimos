import rospy
from google.cloud import speech
from google.cloud.speech import types

class NodeGoogleSpeech:
    def __init__(self):
        self.client = speech.SpeechClient()
        self.recognizer = self.client.streaming_recognize

    def transcribe_audio(self, audio_content):
        audio = speech.RecognitionAudio(content=audio_content)
        config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=16000,
            language_code="en-US"
        )

        try:
            response = self.recognizer(config=config, audio=audio)
            for result in response:
                print(f"Transcript: {result.alternatives[0].transcript}")
        except Exception as e:
            rospy.logerr("Speech recognition failed: {}".format(e))
            return