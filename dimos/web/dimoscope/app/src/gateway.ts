// Runtime-editable gateway address, shared between main.tsx and App.tsx — its own module to avoid a circular import.
import { createContext, useContext } from "react";

export interface GatewayCtx {
  gateway: string; // host:port the transports connect to
  setGateway: (g: string) => void; // persist + reconnect
}

export const GatewayContext = createContext<GatewayCtx>({ gateway: "", setGateway: () => {} });
export const useGateway = () => useContext(GatewayContext);
