/** Only transport-only placeholder lines are omitted; ordinary message text stays intact. */
export function displayText(text: string): string {
  return text.split("\n").filter(line => !/^\s*\[(reasoning|function_call)\]\s*$/.test(line)).join("\n").trim();
}
export function inlineTokens(text: string): { kind: "text" | "strong" | "code"; text: string }[] {
  return text.split(/(\*\*[^*\n]+\*\*|`[^`\n]+`)/g).filter(Boolean).map(part =>
    part.startsWith("**") && part.endsWith("**") ? { kind: "strong", text: part.slice(2,-2) } :
    part.startsWith("`") && part.endsWith("`") ? { kind: "code", text: part.slice(1,-1) } : { kind: "text", text: part });
}
