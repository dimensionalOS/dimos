# Kora Social

![Kora Social](static/brand/kora-banner.png)

Kora Social is a hackathon dashboard for a Unitree Go2 robot that captures its own point of view, drafts X posts with OpenRouter vision models, and lets people ask the robot what it sees through X mentions.

## What it does

- Go2 camera preview, with sample frames for robot-free testing
- Capture-to-draft workflow for X posts
- OpenRouter captions and visual Q&A
- Opt-in people tag suggestions
- Local mention queue with manual inject/poll/run controls
- Optional X posting through `xurl`
- Local stand/rest/stop/drive and Tiny Patrol controls

## Run

From the repo root:

```bash
.venv/bin/python -m uvicorn hackathon.kora_social.app:app --reload --host 127.0.0.1 --port 8787
```

Open `http://127.0.0.1:8787`.

To try the dashboard without a robot connected, start it with sample frames:

```bash
MOCK_ROBOT=1 .venv/bin/python -m uvicorn hackathon.kora_social.app:app --reload --host 127.0.0.1 --port 8787
```

## Configuration

Put local secrets in the repo `.env` file:

```bash
OPENROUTER_API_KEY=...
KORA_SOCIAL_OPENROUTER_MODEL=openai/gpt-4o-mini
```

Useful optional settings:

```bash
KORA_SOCIAL_X_MENTION_QUERY='@DimensionalKora -from:DimensionalKora'
KORA_SOCIAL_X_MENTION_MODE=search
```

`xurl` handles X auth outside the app. Posting is preview-only by default; set `KORA_SOCIAL_X_DRY_RUN=0` to send real posts.

## Mention Commands

```text
@DimensionalKora what do you see?
@DimensionalKora take a picture
```

Mention handling currently supports vision and capture actions. Speech and movement mentions are ignored. Unknown non-movement mentions can be classified through OpenRouter into `vision`, `capture`, or `ignored`.

## Runtime State

Runtime files are intentionally ignored by git:

- `hackathon/kora_social/captures/`
- `hackathon/kora_social/people_tags/`
- `hackathon/kora_social/kora_social.sqlite3`
