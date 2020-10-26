"""
    calibrate(c, xy)

Calibrate a coordinate, transforming it from pixel to real-world
coordinate. `xy` must be a `StaticArrays.SVector{2, Float64}`.
"""
calibrate(c, xy::V2) = c.tform(xy)

"""
    calibrate(c, img)

Calibrate an image, transforming its coordinates from pixel to real-world.
"""
function calibrate(c, img)
    indices = ImageTransformations.autorange(img, c.tform)
    warp(img, c.itform, indices)
end
