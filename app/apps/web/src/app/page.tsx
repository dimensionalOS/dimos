import type { Message } from "@robomoo/shared";
import Image from "next/image";
import Link from "next/link";
import { AddMessage } from "@/components/add-message";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { rpcClient } from "@/lib/orpc";

// Always render fresh — the message list is dynamic.
export const dynamic = "force-dynamic";

export default async function Home() {
  let messages: Message[] = [];
  let error: string | null = null;
  try {
    messages = await rpcClient.messages.list();
  } catch (e) {
    error = e instanceof Error ? e.message : "failed to load messages";
  }

  return (
    <main className="mx-auto flex max-w-2xl flex-col gap-6 px-4 py-12">
      <header className="flex flex-col gap-1">
        <h1 className="font-bold text-3xl tracking-tight">robomoo</h1>
        <p className="text-muted-foreground text-sm">
          Bun + Hono + oRPC + Postgres + object storage, behind a Caddy gateway
          on Railway.
        </p>
        <div className="flex gap-3 text-sm underline">
          <Link href="/frames">→ robot frames</Link>
          <Link href="/map">→ robot map</Link>
        </div>
      </header>

      <AddMessage />

      {error ? (
        <p className="text-destructive text-sm">Error loading messages: {error}</p>
      ) : null}

      <section className="flex flex-col gap-3">
        {messages.map((m) => (
          <Card key={m.id}>
            <CardHeader>
              <CardTitle className="text-sm">{m.authorName}</CardTitle>
              <time className="text-muted-foreground text-xs">
                {new Date(m.createdAt).toLocaleString()}
              </time>
            </CardHeader>
            <CardContent className="flex flex-col gap-3">
              <p className="text-sm">{m.body}</p>
              {m.imageUrl ? (
                <Image
                  alt=""
                  className="rounded-md border"
                  height={320}
                  src={m.imageUrl}
                  unoptimized
                  width={480}
                />
              ) : null}
            </CardContent>
          </Card>
        ))}
        {messages.length === 0 && !error ? (
          <p className="text-muted-foreground text-sm">No messages yet.</p>
        ) : null}
      </section>
    </main>
  );
}
