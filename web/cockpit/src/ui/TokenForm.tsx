import { MAX_TOKEN_LEN } from "@dimos/shared";
import { useState } from "react";
import styles from "./TokenForm.module.css";

/** The login for a relay with auth: shown when the relay rejected the session
 * with auth_failed, with the relay's message (a missing versus an invalid
 * token) above a password field. */
export function TokenForm({ message, onSubmit }: {
  message: string;
  onSubmit: (token: string) => void;
}) {
  const [draft, setDraft] = useState("");
  return (
    <form
      className={styles.form}
      data-testid="token-form"
      onSubmit={(e) => {
        e.preventDefault();
        if (draft !== "") onSubmit(draft);
      }}
    >
      <p className={styles.hint}>This relay requires a viewer token</p>
      <p className={styles.message} data-testid="token-message">{message}</p>
      <input
        type="password"
        className={styles.input}
        data-testid="token-input"
        aria-label="Viewer token"
        autoComplete="off"
        autoFocus
        maxLength={MAX_TOKEN_LEN}
        value={draft}
        onChange={(e) => setDraft(e.target.value)}
      />
      <button
        type="submit"
        className={styles.connect}
        data-testid="token-connect"
        disabled={draft === ""}
      >
        Connect
      </button>
    </form>
  );
}
