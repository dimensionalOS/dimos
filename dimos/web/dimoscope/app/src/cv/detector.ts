// detector.ts — a model-agnostic in-browser object detector. The CV layer talks to this interface,
// so the model (COCO-SSD today; MediaPipe / ONNX Runtime Web later) is a one-file swap. The heavy ML
// deps are dynamic-imported on first load, so they only ship to the browser when CV is turned on.

export interface Box {
  // Frame-pixel coords: the detector's input IS the camera frame, so these map 1:1 to the canvas
  // useVideo draws to (CSS scales the canvas uniformly, so the boxes scale with the image).
  x: number;
  y: number;
  w: number;
  h: number;
  label: string;
  score: number;
}

export interface Detector {
  /** Run detection on one frame (a canvas / ImageBitmap / VideoFrame — any CanvasImageSource). */
  detect(src: CanvasImageSource): Promise<Box[]>;
}

/** TensorFlow.js COCO-SSD — turnkey 80-class object detection on the WebGL/WebGPU backend. */
export async function loadCocoDetector(): Promise<Detector> {
  const [tf, cocoSsd] = await Promise.all([
    import("@tensorflow/tfjs"),
    import("@tensorflow-models/coco-ssd"),
  ]);
  await tf.ready(); // initialise the best available backend (WebGL / WebGPU / WASM)
  const model = await cocoSsd.load();
  return {
    async detect(src) {
      // coco-ssd's typed input union doesn't list every CanvasImageSource; we always pass an
      // HTMLCanvasElement (the work-canvas), which it accepts at runtime — cast over the gap.
      const preds = await model.detect(src as unknown as HTMLCanvasElement);
      return preds.map((p) => ({
        x: p.bbox[0],
        y: p.bbox[1],
        w: p.bbox[2],
        h: p.bbox[3],
        label: p.class,
        score: p.score,
      }));
    },
  };
}
