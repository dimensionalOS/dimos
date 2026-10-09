# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             # Acceptable values differ by the image transport used:
                             # - compressed_image_transport:
                             #     ORIG_PIXFMT; CODEC compressed [COMPRESSED_PIXFMT]
                             #   where:
                             #   - ORIG_PIXFMT is pixel format of the raw image, i.e.
                             #     the content of sensor_msgs/Image/encoding with
                             #     values from include/sensor_msgs/image_encodings.h
                             #   - CODEC is one of [jpeg, png, tiff]
                             #   - COMPRESSED_PIXFMT is only appended for color images
                             #     and is the pixel format used by the compression
                             #     algorithm. Valid values for jpeg encoding are:
                             #     [bgr8, rgb8]. Valid values for png encoding are:
                             #     [bgr8, rgb8, bgr16, rgb16].
                             #   If the field is empty or does not correspond to the
                             #   above pattern, the image is treated as bgr8 or mono8
                             #   jpeg image (depending on the number of channels).
                             # - compressed_depth_image_transport:
                             #     ORIG_PIXFMT; compressedDepth CODEC
                             #   where:
                             #   - ORIG_PIXFMT is pixel format of the raw image, i.e.
                             #     the content of sensor_msgs/Image/encoding with
                             #     values from include/sensor_msgs/image_encodings.h
                             #     It is usually one of [16UC1, 32FC1].
                             #   - CODEC is one of [png, rvl]
                             #   If the field is empty or does not correspond to the
                             #   above pattern, the image is treated as png image.
                             # - Other image transports can store whatever values they
                             #   need for successful decoding of the image. Refer to
                             #   documentation of the other transports for details.

uint8[] data                 # Compressed image buffer
