import { StrictMode } from "react";
import { createRoot } from "react-dom/client";

import { App } from "./App";
import { assertViewModel } from "./session";
import "./styles.css";

async function loadSession() {
  const response = await fetch("./session.json", {
    credentials: "omit",
    cache: "no-store",
    redirect: "error",
  });
  if (!response.ok) {
    throw new Error("The staged session document is unavailable");
  }
  return assertViewModel(await response.json());
}

function renderFailure(message: string) {
  const root = document.getElementById("root");
  if (root) {
    root.innerHTML = `<main class="load-failure"><h1>Session unavailable</h1><p>${message}</p></main>`;
  }
}

loadSession()
  .then((session) => {
    const root = document.getElementById("root");
    if (!root) throw new Error("Viewer root is unavailable");
    createRoot(root).render(
      <StrictMode>
        <App session={session} />
      </StrictMode>,
    );
  })
  .catch(() => renderFailure("The local review document could not be loaded."));
