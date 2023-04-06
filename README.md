# CameraCalibrations

This is a package for camera calibration in Julia.

# How to install
```julia
] add CameraCalibrations
```

# How to use
First we build the calibration object based on the `files`: the image files of the checkerboard, `n_corners`: the number of corners in each of the sides of the checkerboard, and `checker_size`: the physical size of the checker (e.g. in cm).


```julia
using CameraCalibrations
c = Calibration(files, n_corners, checker_size)
```

Then we can use that object to calibrate pixel coordinates and/or images, given the extrinsic image we want to refer to:
```julia
i1 = RowCol(1.2, 3.4) # a cartesian index in image-pixel coordinates
xyz = c(i1, 1) # convert to real-world coordinates of the first image
i2 = c(xyz, 1) # convert back to pixel coordinates
i2 ≈ i1 # true
```

# Why
## use this
1. This is currently (1/4/2023) the only package for camera calibration in Julia.
2. It allows for calibration using any number of image (even one).

## not use this
1. Detecting the corners of the checkerboards and calculating the camera-model's parameters is done used [OpenCV](https://opencv.org/).
2. You can't supply it with in-memory images, just file names.
