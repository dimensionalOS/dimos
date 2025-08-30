import * as React from "react";
import * as ReactDOMClient from "react-dom/client";

import App from "./App";

export function initializeApp(element: HTMLElement): void {
  const root = ReactDOMClient.createRoot(element);
  root.render(React.createElement(App));
}
