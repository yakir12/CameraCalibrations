# CameraCalibrations

This is a package for camera calibration in Julia. This will only work if you have Matlab™ and you Matlab™'s `Computer Vision System` toolbox installed.

# How to install
```julia
] add CameraCalibrations
```

# How to use
First we build the calibration object based on the checker size, the extrinsic image, and intrinsic images:
```julia
using CameraCalibrations
c = buildcalibration(3, extrinsic, intrinsic)
```

Then we can use that object to calibrate pixel coordinates and/or images:
```julia
xy = calibrate(c, SVector(1.1, 2.2))
imgw = calibrate(c, img)
```

# Why
## use this
1. This is currently (26/10/2020) the only package for camera calibration in Julia.
2. It allows for calibration using only one (extrinsic) image as well as both extrinsic and intrinsic images.
3. The calibration objects are native Julia objects (an object from `CoordinateTransformations` or `Interpolations`), so you can save them.

## not use this
1. It's just a poor wrapper around Matlab's camera calibration toolbox (so without it you can't use this).
2. You can't supply it with in-memory images, just file names.
3. Error handling within Matlab is suboptimal.
4. It's not optimized for speed or anything really.
