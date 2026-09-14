// The viewer token for a relay started with --auth-file, kept across reloads
// so the single connect() in main.tsx can send it; "log out" forgets it.
const KEY = "dimos.cockpit.token";

export function readToken(): string | null {
  return localStorage.getItem(KEY);
}

export function storeToken(token: string): void {
  localStorage.setItem(KEY, token);
}

export function clearToken(): void {
  localStorage.removeItem(KEY);
}
