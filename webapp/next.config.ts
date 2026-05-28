import type { NextConfig } from "next";

const nextConfig: NextConfig = {
  reactStrictMode: true,
  // Allow any ngrok tunnel to hit the dev server (real-iPhone testing).
  // Wildcard so you never have to update this when the tunnel URL changes.
  allowedDevOrigins: ["*.ngrok-free.app"],
};

export default nextConfig;
