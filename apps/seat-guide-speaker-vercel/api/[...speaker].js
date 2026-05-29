const messages = globalThis.__seatGuideSpeakerMessages || new Map();
globalThis.__seatGuideSpeakerMessages = messages;

function json(res, status, body) {
  res.statusCode = status;
  res.setHeader("content-type", "application/json; charset=utf-8");
  res.end(JSON.stringify(body));
}

function sanitizeDevice(value) {
  const device = String(value || "go2-demo")
    .trim()
    .replace(/[^a-zA-Z0-9_-]/g, "-")
    .slice(0, 80);
  return device || "go2-demo";
}

async function readBody(req) {
  const chunks = [];
  for await (const chunk of req) chunks.push(chunk);
  const raw = Buffer.concat(chunks).toString("utf8");
  return raw ? JSON.parse(raw) : {};
}

async function handleSpeak(req, res) {
  if (req.method !== "POST") {
    json(res, 405, { ok: false, error: "method_not_allowed" });
    return;
  }

  try {
    const body = await readBody(req);
    const text = String(body.text || "").trim();
    if (!text) {
      json(res, 400, { ok: false, error: "missing_text" });
      return;
    }
    const device = sanitizeDevice(body.device);
    const message = {
      id: `${Date.now()}-${Math.random().toString(16).slice(2)}`,
      device,
      text: text.slice(0, 800),
      createdAt: new Date().toISOString(),
    };
    messages.set(device, message);
    json(res, 200, { ok: true, storage: "memory", message });
  } catch (error) {
    json(res, 500, { ok: false, error: String(error.message || error) });
  }
}

function handleLatest(req, res) {
  if (req.method !== "GET") {
    json(res, 405, { ok: false, error: "method_not_allowed" });
    return;
  }
  const url = new URL(req.url, `https://${req.headers.host || "localhost"}`);
  const device = sanitizeDevice(url.searchParams.get("device"));
  json(res, 200, { ok: true, device, message: messages.get(device) || null });
}

export default async function handler(req, res) {
  const url = new URL(req.url, `https://${req.headers.host || "localhost"}`);
  const route = url.pathname.split("/").filter(Boolean).at(-1);
  if (route === "speak") {
    await handleSpeak(req, res);
    return;
  }
  if (route === "latest") {
    handleLatest(req, res);
    return;
  }
  json(res, 404, { ok: false, error: "not_found" });
}
