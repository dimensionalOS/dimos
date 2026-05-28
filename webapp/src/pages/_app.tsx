import "@/styles/globals.css";
import type { AppProps } from "next/app";
import Head from "next/head";

export default function App({ Component, pageProps }: AppProps) {
  return (
    <>
      <Head>
        {/* viewport-fit=cover lets the gradient extend under the notch/home bar.
            maximum-scale=1 stops iOS auto-zoom when focusing the (hidden) text input. */}
        <meta
          name="viewport"
          content="width=device-width, initial-scale=1, maximum-scale=1, viewport-fit=cover"
        />
        <meta name="theme-color" content="#0a0b0e" />
      </Head>
      <Component {...pageProps} />
    </>
  );
}
