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
};

export default nextConfig;
