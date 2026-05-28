# SeatGuide Speaker Vercel App

This app lets an iPhone mounted on the Go2 act as the SeatGuide speaker.

Flow:

1. iPhone opens the deployed page with cellular data.
2. The page polls `/api/latest?device=go2-demo`.
3. Mac/DimOS posts arrival text to `/api/speak`.
4. The iPhone speaks the latest message with the local browser speaker.

This minimal Vercel version stores only the latest message in serverless memory.
It is enough for quick demos, but can lose messages on cold starts or instance
changes.

## Deploy

Create a Vercel project from this directory:

```bash
cd apps/seat-guide-speaker-vercel
npm install
npx vercel
```

No Redis or database is required for the quick demo version.

## iPhone

Open:

```text
https://<your-vercel-domain>/?device=go2-demo
```

Tap `Enable speaker`. Keep Safari open and unlocked.

## Mac Test

```bash
curl -X POST "https://<your-vercel-domain>/api/speak" \
  -H "authorization: Bearer $SPEAKER_API_TOKEN" \
  -H "content-type: application/json" \
  -d '{"device":"go2-demo","text":"我已经到了, 请坐。"}'
```
