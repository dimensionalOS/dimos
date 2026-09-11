import { createContext } from "react";
import type { Session } from "@dimos/sdk";
export const ViewerSession = createContext<Session | null>(null);
