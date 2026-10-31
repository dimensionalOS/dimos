// A module's or the blueprint's source, over the graph, scrolled to its line.
import { Button, CodeBlock, EditorNote, Spacer, useDesktopEditor } from "dimos-react";
import { useEffect, useState } from "react";
import { errorText, getJson } from "./api.ts";

export interface Where {
  file: string;
  line: number;
}

export function CodePane({ where, onClose }: { where: Where; onClose: () => void }) {
  const [text, setText] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);
  const editor = useDesktopEditor();
  useEffect(() => {
    let current = true;
    setText(null);
    setError(null);
    getJson<{ text: string }>(`source?file=${encodeURIComponent(where.file)}`).then(
      (answer) => current && setText(answer.text),
      (caught) => current && setError(errorText(caught)),
    );
    return () => {
      current = false;
    };
  }, [where.file]);
  return (
    <div id="code">
      <div className="code-head">
        <span className="file" title={where.file}>{where.file}</span>
        <span className="note">line {where.line}</span>
        <Spacer />
        {editor.open && (
          <Button
            size="sm"
            onClick={() => editor.open?.(where.file, where.line)}
            data-bp-open-editor
          >
            Open in editor
          </Button>
        )}
        <Button size="sm" onClick={onClose}>Hide code</Button>
      </div>
      <EditorNote status={editor.status} />
      {text === null ? <div className="note">{error ?? "loading…"}</div> : (
        <CodeBlock
          code={text}
          language="py"
          lineNumbers
          header={false}
          fill
          highlightLine={where.line}
        />
      )}
    </div>
  );
}
