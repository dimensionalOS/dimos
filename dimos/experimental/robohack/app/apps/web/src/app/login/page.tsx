"use client";

import { useRouter } from "next/navigation";
import { type FormEvent, useState } from "react";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { signIn, signUp } from "@/lib/auth-client";

export default function LoginPage() {
  const router = useRouter();
  const [name, setName] = useState("");
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [mode, setMode] = useState<"signin" | "signup">("signup");
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState<string | null>(null);

  async function onSubmit(e: FormEvent) {
    e.preventDefault();
    setBusy(true);
    setError(null);
    const result =
      mode === "signup"
        ? await signUp.email({ name, email, password })
        : await signIn.email({ email, password });
    setBusy(false);
    if (result.error) {
      setError(result.error.message ?? "authentication failed");
      return;
    }
    router.push("/");
    router.refresh();
  }

  return (
    <main className="mx-auto flex max-w-sm flex-col gap-6 px-4 py-16">
      <Card>
        <CardHeader>
          <CardTitle>{mode === "signup" ? "Create account" : "Sign in"}</CardTitle>
        </CardHeader>
        <CardContent>
          <form className="flex flex-col gap-3" onSubmit={onSubmit}>
            {mode === "signup" ? (
              <Input
                onChange={(e) => setName(e.target.value)}
                placeholder="Name"
                required
                value={name}
              />
            ) : null}
            <Input
              onChange={(e) => setEmail(e.target.value)}
              placeholder="Email"
              required
              type="email"
              value={email}
            />
            <Input
              onChange={(e) => setPassword(e.target.value)}
              placeholder="Password (8+ chars)"
              required
              type="password"
              value={password}
            />
            {error ? <p className="text-destructive text-sm">{error}</p> : null}
            <Button disabled={busy} type="submit">
              {busy ? "…" : mode === "signup" ? "Sign up" : "Sign in"}
            </Button>
          </form>
          <Button
            className="mt-2 w-full"
            onClick={() => setMode(mode === "signup" ? "signin" : "signup")}
            size="sm"
            type="button"
            variant="link"
          >
            {mode === "signup"
              ? "Already have an account? Sign in"
              : "Need an account? Sign up"}
          </Button>
        </CardContent>
      </Card>
    </main>
  );
}
