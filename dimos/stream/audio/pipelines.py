from dimos.stream.audio.node_microphone import SounddeviceAudioSource
from dimos.stream.audio.node_normalizer import AudioNormalizer
from dimos.stream.audio.node_volume_monitor import monitor
from dimos.stream.audio.node_key_recorder import KeyRecorder
from dimos.stream.audio.node_output import SounddeviceAudioOutput
from dimos.stream.audio.stt.node_whisper import WhisperNode
from dimos.stream.audio.tts.node_openai import OpenAITTSNode
from dimos.stream.audio.text.node_stdout import TextPrinterNode


def stt():
    # Create microphone source, recorder, and audio output
    mic = SounddeviceAudioSource()
    normalizer = AudioNormalizer()
    recorder = KeyRecorder(always_subscribe=True)
    whisper_node = WhisperNode()  # Assign to global variable

    # Connect audio processing pipeline
    normalizer.consume_audio(mic.emit_audio())
    recorder.consume_audio(normalizer.emit_audio())
    monitor(recorder.emit_audio())
    whisper_node.consume_audio(recorder.emit_recording())

    user_text_printer = TextPrinterNode(prefix="USER: ")
    user_text_printer.consume_text(whisper_node.emit_text())

    return whisper_node


def tts():
    tts_node = OpenAITTSNode()
    agent_text_printer = TextPrinterNode(prefix="AGENT: ")
    agent_text_printer.consume_text(tts_node.emit_text())

    response_output = SounddeviceAudioOutput(sample_rate=24000)
    response_output.consume_audio(tts_node.emit_audio())

    return tts_node
