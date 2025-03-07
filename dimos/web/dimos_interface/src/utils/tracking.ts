declare global {
  interface Window {
    umami: {
      track: (event: string, data?: Record<string, unknown>) => Promise<void>;
    };
  }
}

export const track = (cmd: string, ...args: string[]) => {
  if (window.umami) {
    window.umami.track(cmd, {
      args: args.join(' '),
    });
  }
};
