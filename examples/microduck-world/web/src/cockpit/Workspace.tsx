import { createContext, useContext, useState, useEffect, type ReactNode } from "react";

type Hidden = { id: string; title: string; restore: () => void };
const WorkspaceContext = createContext<{ hide: (entry: Hidden) => void; forget: (id: string) => void } | null>(null);
export const useWorkspace = () => useContext(WorkspaceContext);
export function Workspace({ children }: { children: ReactNode }) {
  const [replaced, setReplaced] = useState(false);
  useEffect(() => {
    const stop = () => setReplaced(true);
    globalThis.addEventListener("microduck-session-replaced", stop);
    return () => globalThis.removeEventListener("microduck-session-replaced", stop);
  }, []);
  const [hidden, setHidden] = useState<Hidden[]>([]);
  const [theme, setTheme] = useState(() => {
    try { return localStorage.getItem("microduck-theme") === "light" ? "light" : "dark"; } catch { return "dark"; }
  });
  return <WorkspaceContext.Provider value={{
    hide: entry => setHidden(items => [...items.filter(i => i.id !== entry.id), entry]),
    forget: id => setHidden(items => items.filter(i => i.id !== id)),
  }}>
    <div className="mw-workspace" data-theme={theme}>
      <div className="mw-workspace-tools">
        {replaced && <span role="alert">Your connection moved to another tab. This tab has stopped reconnecting. Close the other tab and reload here to resume.</span>}
        {hidden.map(entry => <button key={entry.id} onClick={() => { entry.restore(); setHidden(items => items.filter(i => i.id !== entry.id)); }}>{entry.title} ↗</button>)}
        <button onClick={() => globalThis.dispatchEvent(new Event("microduck-arrange"))}>Reset panels</button>
        <button role="switch" aria-label="Light mode" aria-checked={theme === "light"} onClick={() => {
          const next = theme === "dark" ? "light" : "dark";
          setTheme(next); try { localStorage.setItem("microduck-theme", next); } catch { /* Storage is optional. */ }
        }}>{theme === "dark" ? "☀ Light" : "☾ Dark"}</button>
      </div>
      {children}
    </div>
  </WorkspaceContext.Provider>;
}
