/// <reference types="vite/client" />

// draco3d ships no type declarations; we only call createDecoderModule + the decoder handle (any).
declare module "draco3d" {
  export function createDecoderModule(config?: unknown): Promise<any>;
  export function createEncoderModule(config?: unknown): Promise<any>;
}
