import type { NextConfig } from "next";

const nextConfig: NextConfig = {
  reactStrictMode: true,
  // Self-contained server bundle at .next/standalone for a slim Docker image.
  output: "standalone",
  transpilePackages: ["@robomoo/shared", "@robomoo/api"],
  images: {
    // Presigned image URLs come from the object-storage endpoint. Allow any
    // host for the sample; tighten to the bucket host in a real app.
    remotePatterns: [{ protocol: "https", hostname: "**" }],
  },
  // Dev-only: the Caddy gateway provides same-origin /rpc + /api in prod (and
  // via `bun run dev`). When running the web standalone (e.g. on macOS where the
  // gateway's host-networking container is unavailable), proxy those paths to
  // the server so the browser stays same-origin. No-op in production.
  async rewrites() {
    if (process.env.NODE_ENV === "production") return [];
    const server = process.env.RPC_INTERNAL_URL ?? "http://localhost:4471";
    return [
      { source: "/rpc/:path*", destination: `${server}/rpc/:path*` },
      { source: "/api/:path*", destination: `${server}/api/:path*` },
    ];
  },
};

export default nextConfig;
