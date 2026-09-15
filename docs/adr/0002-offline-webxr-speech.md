# Synthesize WebXR prompts on the computer

Collection prompts use opt-in, offline Kokoro INT8 synthesis in a reusable DimOS
module and ordinary Web Audio playback in the browser. WebXR does not provide
a speech-synthesis API, and browser-native speech depends on available voices.
Keeping synthesis on the computer provides the same voice across headset
browsers and keeps model downloads and inference off the headset. A local HTTP
endpoint returns WAV audio; collection status and video keep their existing
WebSocket path. TTS is disabled by default and requires explicit dependency
installation. An enabled module downloads and verifies missing default assets
during its build step before collection starts; cached assets support offline use.

References: [WebXR Device API](https://www.w3.org/TR/webxr/),
[Web Audio API](https://www.w3.org/TR/webaudio-1.0/),
[Web Speech API](https://webaudio.github.io/web-speech-api/),
[Kokoro ONNX](https://github.com/thewh1teagle/kokoro-onnx).
