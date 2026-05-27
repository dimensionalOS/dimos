// In-browser CLIP text encoding for semantic search over recorded frames.
//
// Uses the SAME model dimos embeds images with — openai/clip-vit-base-patch32
// (here the ONNX port `Xenova/clip-vit-base-patch32`) — so the 512-d text and
// image vectors share one space. Image vectors come pre-normalized from dimos;
// we L2-normalize the text vector, then cosine similarity = a plain dot product.
//
// The model (~tens of MB, quantized) downloads from the HF hub on first query
// and is cached by the browser thereafter. No server-side ML.

import {
  AutoTokenizer,
  CLIPTextModelWithProjection,
  env,
} from "@huggingface/transformers";

env.allowLocalModels = false; // always fetch from the hub/CDN

const MODEL_ID = "Xenova/clip-vit-base-patch32";

let tokenizerP: Promise<unknown> | null = null;
let modelP: Promise<unknown> | null = null;

async function ensureModel() {
  tokenizerP ??= AutoTokenizer.from_pretrained(MODEL_ID);
  modelP ??= CLIPTextModelWithProjection.from_pretrained(MODEL_ID);
  // biome-ignore lint/suspicious/noExplicitAny: transformers.js types are loose
  const [tokenizer, model] = (await Promise.all([tokenizerP, modelP])) as any[];
  return { tokenizer, model };
}

// Encode a text query into a normalized 512-d CLIP vector.
export async function embedText(query: string): Promise<number[]> {
  const { tokenizer, model } = await ensureModel();
  const inputs = tokenizer([query], { padding: true, truncation: true });
  const { text_embeds } = await model(inputs);
  const v = Array.from(text_embeds.data as Float32Array) as number[];
  let norm = 0;
  for (const x of v) norm += x * x;
  norm = Math.sqrt(norm) || 1;
  return v.map((x) => x / norm);
}

// Cosine similarity for normalized vectors (= dot product). Falls back to a full
// cosine if lengths differ or a vector isn't normalized.
export function cosine(a: number[], b: number[]): number {
  const n = Math.min(a.length, b.length);
  let dot = 0;
  for (let i = 0; i < n; i++) dot += (a[i] ?? 0) * (b[i] ?? 0);
  return dot;
}
