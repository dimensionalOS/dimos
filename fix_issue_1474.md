```python
# Import necessary libraries
import rospy
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import google.cloud.speech as speech
from google.cloud.speech import types

# Initialize Google Cloud Speech client
client = speech.SpeechClient()

# Define a callback function for audio data
def audio_callback(audio_data):
    # Convert ROS AudioData message to bytes
    audio_content = audio_data.data

    # Create a recognition audio object
    audio = types.RecognitionAudio(content=audio_content)

    # Create a configuration object
    config = types.RecognitionConfig(
        encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=16000,
        language_code='en-US')

    # Perform the speech recognition
    response = client.recognize(config=config, audio=audio)

    # Extract the transcription
    for result in response.results:
        transcription = result.alternatives[0].transcript
        rospy.loginfo("Transcription: {}".format(transcription))

        # Publish the transcription to a ROS topic
        transcription_pub.publish(String(data=transcription))

# Initialize ROS node
rospy.init_node('realtime_transcription')

# Create a publisher for the transcription
transcription_pub = rospy.Publisher('transcription', String, queue_size=10)

# Subscribe to the audio topic
rospy.Subscriber('audio_input', AudioData, audio_callback)

# Spin to keep the node running
rospy.spin()
```

### Explanation of Changes:
1. **Google Cloud Speech Client Initialization**: The Google Cloud Speech client is initialized to handle the speech-to-text conversion.
2. **Audio Callback Function**: This function processes incoming audio data from the ROS topic, converts it to bytes, and sends it to the Google Cloud Speech API for transcription.
3. **ROS Node Setup**: The ROS node is initialized, and a subscriber is set up to listen to the `audio_input` topic for audio data. A publisher is also set up to publish the transcription to the `transcription` topic.
4. **Main Loop**: The `rospy.spin()` function keeps the node running and listening for incoming audio data.

### Test Cases:
1. **Publish Audio Data**: Ensure that audio data is being published to the `audio_input` topic.
2. **Check Transcription Output**: Verify that the transcription is being published to the `transcription` topic and is correct.
3. **Error Handling**: Add error handling for network issues or API failures to ensure robustness.