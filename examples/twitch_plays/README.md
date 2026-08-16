# Twitch Chat Integration

Connect a Twitch channel's chat to DimOS as a module.

## Setup

1. **Get Twitch credentials** from [twitchtokengenerator.com](https://twitchtokengenerator.com/):
   - Select "Custom Scope Token"
   - Choose scopes: `chat:read`, `chat:edit`
   - Copy the Access Token

2. **Set environment variables**:
   ```bash
   export DIMOS_TWITCH_TOKEN=oauth:your_access_token_here
   export DIMOS_CHANNEL_NAME=your_twitch_channel
   ```

3. **Install twitchio** (not in DimOS base deps):
   ```bash
   uv pip install twitchio
   ```

## Run

```bash
dimos run unitree-go2-twitch --robot-ip 192.168.123.161
```

## Streams

- `raw_messages` — every chat message as a `TwitchMessage`
- `filtered_messages` — messages matching configured regex patterns and filters

## Filters

```python
TwitchChat.blueprint(
    patterns=[r"^!(?:forward|back|left|right)"],  # regex on content
    filter_is_mod=True,                            # mods only
    filter_is_subscriber=True,                     # subscribers only
    filter_author=lambda name: name != "nightbot", # exclude bots
    filter_content=lambda text: len(text) < 200,   # reject spam
)
```

## Local Testing

```python
from dimos.stream.twitch.module import TwitchChat

chat = TwitchChat()
chat.start()  # runs in local-only mode without credentials

chat.inject_message("!forward", author="user1")
chat.inject_message("hello", author="user2")
```
