"use client";

import {
  createContext,
  type ReactNode,
  useCallback,
  useContext,
  useEffect,
  useState,
} from "react";
import { SEPOLIA_CHAIN_ID } from "./chain";

interface WalletState {
  address: `0x${string}` | null;
  chainId: number | null;
  connecting: boolean;
  error: string | null;
  hasWallet: boolean;
  onSepolia: boolean;
  connect: () => Promise<void>;
  disconnect: () => void;
  ensureSepolia: () => Promise<boolean>;
}

const WalletContext = createContext<WalletState | null>(null);

const SEPOLIA_HEX = "0xaa36a7"; // 11155111

export function WalletProvider({ children }: { children: ReactNode }) {
  const [address, setAddress] = useState<`0x${string}` | null>(null);
  const [chainId, setChainId] = useState<number | null>(null);
  const [connecting, setConnecting] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [hasWallet, setHasWallet] = useState(false);

  useEffect(() => {
    const eth = window.ethereum;
    if (!eth) return;
    setHasWallet(true);
    eth
      .request({ method: "eth_accounts" })
      .then((a) => {
        const addr = (a as string[])?.[0];
        if (addr) setAddress(addr as `0x${string}`);
      })
      .catch(() => {});
    eth
      .request({ method: "eth_chainId" })
      .then((c) => setChainId(Number.parseInt(c as string, 16)))
      .catch(() => {});

    const onAccounts = (...args: unknown[]) => {
      const accounts = args[0] as string[] | undefined;
      setAddress((accounts?.[0] as `0x${string}`) ?? null);
    };
    const onChain = (...args: unknown[]) => {
      setChainId(Number.parseInt(args[0] as string, 16));
    };
    eth.on?.("accountsChanged", onAccounts);
    eth.on?.("chainChanged", onChain);
    return () => {
      eth.removeListener?.("accountsChanged", onAccounts);
      eth.removeListener?.("chainChanged", onChain);
    };
  }, []);

  const connect = useCallback(async () => {
    const eth = window.ethereum;
    if (!eth) {
      setError("No browser wallet found. Install MetaMask to continue.");
      return;
    }
    setConnecting(true);
    setError(null);
    try {
      const a = (await eth.request({
        method: "eth_requestAccounts",
      })) as string[];
      setAddress((a?.[0] as `0x${string}`) ?? null);
      const c = (await eth.request({ method: "eth_chainId" })) as string;
      setChainId(Number.parseInt(c, 16));
    } catch (e) {
      setError(e instanceof Error ? e.message : "failed to connect wallet");
    } finally {
      setConnecting(false);
    }
  }, []);

  const disconnect = useCallback(() => setAddress(null), []);

  const ensureSepolia = useCallback(async (): Promise<boolean> => {
    const eth = window.ethereum;
    if (!eth) return false;
    if (chainId === SEPOLIA_CHAIN_ID) return true;
    try {
      await eth.request({
        method: "wallet_switchEthereumChain",
        params: [{ chainId: SEPOLIA_HEX }],
      });
      setChainId(SEPOLIA_CHAIN_ID);
      return true;
    } catch {
      setError("Switch your wallet to the Ethereum Sepolia testnet.");
      return false;
    }
  }, [chainId]);

  return (
    <WalletContext.Provider
      value={{
        address,
        chainId,
        connecting,
        error,
        hasWallet,
        onSepolia: chainId === SEPOLIA_CHAIN_ID,
        connect,
        disconnect,
        ensureSepolia,
      }}
    >
      {children}
    </WalletContext.Provider>
  );
}

export function useWallet(): WalletState {
  const ctx = useContext(WalletContext);
  if (!ctx) throw new Error("useWallet must be used within a WalletProvider");
  return ctx;
}
