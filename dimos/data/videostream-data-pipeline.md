# UNDER DEVELOPMENT 🚧🚧🚧
# Example data pipeline from video stream implementation

```bash
   from dimos.stream.videostream import VideoStream
   from dimos.data.data_pipeline import DataPipeline

   # init video stream from the camera source
   video_stream = VideoStream(source=0)

   # init data pipeline with desired processors enabled, max workers is 4 by default
   # depth only implementation
   pipeline = DataPipeline(
       video_stream=video_stream,
       run_depth=True,
       run_labels=False,
       run_pointclouds=False,
       run_segmentations=False
   )

   try:
       # Run pipeline
       pipeline.run()
   except KeyboardInterrupt:
       # Handle interrupt
       print("Pipeline interrupted by user.")
   finally:
       # Release the video capture
       video_stream.release()
```
