"use client";

import Link from "next/link";
import { useRouter } from "next/navigation";
import { type FormEvent, useState } from "react";
import { Button } from "@/components/ui/button";
import { Card, CardContent } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { signOut, useSession } from "@/lib/auth-client";
import { rpcClient } from "@/lib/orpc";

export function AddMessage() {
  const { data: session, isPending } = useSession();
  const router = useRouter();
  const [body, setBody] = useState("");
  const [file, setFile] = useState<File | null>(null);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState<string | null>(null);

  if (isPending) {
    return null;
  }

  if (!session) {
    return (
      <Card>
        <CardContent className="flex items-center justify-between py-1">
          <span className="text-muted-foreground text-sm">
            Sign in to post a message.
          </span>
          <Button asChild size="sm">
            <Link href="/login">Sign in</Link>
          </Button>
        </CardContent>
      </Card>
    );
  }

  async function onSubmit(e: FormEvent) {
    e.preventDefault();
    setBusy(true);
    setError(null);
    try {
      let imageKey: string | null = null;
      if (file) {
        const form = new FormData();
        form.append("file", file);
        const res = await fetch("/api/upload/image", {
          method: "POST",
          body: form,
          credentials: "include",
        });
        if (!res.ok) {
          throw new Error(`upload failed (${res.status})`);
        }
        imageKey = ((await res.json()) as { key: string }).key;
      }
      await rpcClient.messages.add({ body, imageKey });
      setBody("");
      setFile(null);
      router.refresh();
    } catch (err) {
      setError(err instanceof Error ? err.message : "failed to post");
    } finally {
      setBusy(false);
    }
  }

  return (
    <Card>
      <CardContent>
        <form className="flex flex-col gap-3" onSubmit={onSubmit}>
          <div className="flex items-center justify-between">
            <span className="text-muted-foreground text-sm">
              Signed in as {session.user.name}
            </span>
            <Button
              onClick={() => signOut().then(() => router.refresh())}
              size="sm"
              type="button"
              variant="ghost"
            >
              Sign out
            </Button>
          </div>
          <Input
            onChange={(e) => setBody(e.target.value)}
            placeholder="Say something…"
            required
            value={body}
          />
          <Input
            accept="image/*"
            onChange={(e) => setFile(e.target.files?.[0] ?? null)}
            type="file"
          />
          {error ? <p className="text-destructive text-sm">{error}</p> : null}
          <Button disabled={busy || body.length === 0} type="submit">
            {busy ? "Posting…" : "Post"}
          </Button>
        </form>
      </CardContent>
    </Card>
  );
}
