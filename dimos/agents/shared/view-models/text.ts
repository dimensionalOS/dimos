/** Removes control characters from text that came from a tool or a module. */
export function clean(text: string): string {
  return text.replace(/[\x00-\x1f\x7f-\x9f]/g, " ");
}

/** Removes control characters except tabs and line breaks from text rendered as Markdown. */
export function cleanMultiline(text: string): string {
  return text.replace(/[\x00-\x08\x0b-\x1f\x7f-\x9f]/g, " ");
}
