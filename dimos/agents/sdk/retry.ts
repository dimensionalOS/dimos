import type { Client } from "#dimos/agents/sdk/main.ts";
import { outcomeUnknown } from "#dimos/agents/sdk/failures.ts";

/** Only for RPCs whose server deduplicates this request ID. A lost reply permits one replay of the same args. */
export async function retryByRequestId<A extends { readonly request: string }, T>(
  client: Client,
  args: A,
  invoke: (sameArgs: A) => Promise<T>,
): Promise<T> {
  try {
    return await Promise.race([invoke(args), client.ended]);
  } catch (error) {
    if (!outcomeUnknown(error)) throw error;
    await client.whenLive();
    return Promise.race([invoke(args), client.ended]);
  }
}
