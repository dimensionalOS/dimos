import { createConnection } from "node:net";

/** Whether anything answers on the socket within `timeoutMs`: a bare connect, no protocol. */
export async function reachable(socketPath: string, timeoutMs: number): Promise<boolean> {
  return new Promise((resolve) => {
    const socket = createConnection(socketPath);
    const done = (answer: boolean) => {
      socket.destroy();
      resolve(answer);
    };
    socket.once("connect", () => done(true));
    socket.once("error", () => done(false));
    socket.setTimeout(timeoutMs, () => done(false));
  });
}
