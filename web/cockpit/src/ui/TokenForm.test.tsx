// @vitest-environment happy-dom
import { MAX_TOKEN_LEN } from "@dimos/shared";
import { afterEach, beforeEach, describe, expect, it } from "vitest";
import { act } from "react";
import { createRoot, type Root } from "react-dom/client";
import { TokenForm } from "./TokenForm.tsx";

(globalThis as { IS_REACT_ACT_ENVIRONMENT?: boolean }).IS_REACT_ACT_ENVIRONMENT = true;

// React tracks controlled inputs through the value setter; go around it.
const setValue = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, "value")?.set;

describe("TokenForm", () => {
  let container: HTMLElement;
  let root: Root;
  let submitted: string[];

  beforeEach(() => {
    container = document.createElement("div");
    document.body.appendChild(container);
    root = createRoot(container);
    submitted = [];
    act(() =>
      root.render(
        <TokenForm message="missing viewer token" onSubmit={(token) => submitted.push(token)} />,
      )
    );
  });

  afterEach(() => {
    act(() => root.unmount());
    container.remove();
  });

  function find<T extends HTMLElement>(testId: string): T {
    return container.querySelector<T>(`[data-testid="${testId}"]`)!;
  }

  function type(text: string): void {
    act(() => {
      const input = find<HTMLInputElement>("token-input");
      setValue?.call(input, text);
      input.dispatchEvent(new Event("input", { bubbles: true }));
    });
  }

  function submit(): void {
    act(() => {
      find<HTMLFormElement>("token-form").dispatchEvent(
        new Event("submit", { bubbles: true, cancelable: true }),
      );
    });
  }

  it("shows the relay's message and hides what is typed", () => {
    expect(find("token-message").textContent).toBe("missing viewer token");
    expect(find<HTMLInputElement>("token-input").type).toBe("password");
    expect(find<HTMLInputElement>("token-input").maxLength).toBe(MAX_TOKEN_LEN);
  });

  it("submits the exact token and refuses an empty one", () => {
    expect(find<HTMLButtonElement>("token-connect").disabled).toBe(true);
    submit();
    expect(submitted).toEqual([]);
    type("  tok-en  ");
    expect(find<HTMLButtonElement>("token-connect").disabled).toBe(false);
    submit();
    expect(submitted).toEqual(["  tok-en  "]);
  });
});
