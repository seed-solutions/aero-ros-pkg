# aero_sensors

## depth_camera

Downsampling utility for depth camera (Kinect2 or Xtion).

### samples

#### points_sample

Simple points reader.

#### points_compressed_sample

Simple scaled points reader.

#### image_sample

Simple image reader.

#### image_centers_sample

Finding saliency boxes in image and calc centroid in 3D.

#### image_bounds_sample

Finding bounding boxes of each clusters in points.


## ps4eye

Utilities for ps4eye stereo camera.

### set_ps4eye_autogain.sh

Ps4eye can be set to auto gain mode via `v4l2-ctl`.
This is sample script to control gain mode.
