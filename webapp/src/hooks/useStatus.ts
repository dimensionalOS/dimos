import { useEffect, useState } from "react";
import * as dimos from "@/lib/dimos";

/** Poll /unitree/status for the connection indicator. */
export function useStatus(intervalMs = 5000) {
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    let alive = true;
    const tick = async () => {
      const { connected } = await dimos.getStatus();
      if (alive) setConnected(connected);
    };
    tick();
    const id = setInterval(tick, intervalMs);
    return () => {
      alive = false;
      clearInterval(id);
    };
  }, [intervalMs]);

  return connected;
}
