// Browser client: talks to the SimStudio VLM backend server
// so your OpenAI key never ships to the browser.

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

export async function requestVlmDecision({ endpoint, model, prompt, imageBase64, context, messages }) {
  const payload = JSON.stringify({ model, prompt, imageBase64, context, messages });
  const maxAttempts = 6;
  let lastError = null;

  for (let attempt = 1; attempt <= maxAttempts; attempt += 1) {
    const res = await fetch(endpoint, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: payload,
    });
    if (res.ok) return await res.json();

    const text = await res.text().catch(() => "");
    const isRetryable = res.status === 429 || res.status === 503 || res.status === 504;
    if (!isRetryable || attempt >= maxAttempts) {
      throw new Error(`VLM request failed (${res.status}): ${text || res.statusText}`);
    }

    lastError = new Error(`retryable status ${res.status}`);
    const retryAfter = Number(res.headers.get("retry-after") || 0);
    const retryAfterMs = retryAfter > 0 ? retryAfter * 1000 : 0;
    const backoffMs = Math.min(12000, 700 * 2 ** (attempt - 1));
    const jitterMs = Math.floor(Math.random() * 350);
    await sleep(Math.max(retryAfterMs, backoffMs) + jitterMs);
  }

  throw lastError || new Error("VLM request failed");
}


