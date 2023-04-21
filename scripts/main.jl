using Revise, CameraCalibrations

n_corners = (6, 8)
dir = "straight"
# n_corners = (6, 9)
# dir = "negative"
# dir = "../CameraCalibrations/test/example"
files = readdir(dir; join=true)
checker_size = 1
c = Calibration(files, n_corners, checker_size)

n, ϵ... = calculate_errors(c)
plot(c)

# using ImageView

using Statistics, LinearAlgebra
using FileIO, StaticArrays, CoordinateTransformations, Rotations, PaddedViews
using ImageTransformations

function lens_distortion(v, k)
    k == 0 && return v
    r² = LinearAlgebra.norm_sqr(v)
    if k < 0 && r² > -1/3/k
        # throw(ArgumentError("real-world coordinates outside lens distortion model (the radial distortion of the lens is too strong)"))
    end
    radial = 1 + k*r²
    radial*v
end

n_corners = (4, 9)
tf = isodd.(sum.(Tuple.(CartesianIndices(n_corners .+ 1))))
img0 = repeat(tf, inner = (100, 100))
w = 10
img = PaddedView(true, img0, size(img0) .+ 2w, (w, w) .+ 1)
# imshow(img)
using GLMakie
n = 1001
fig = Figure()
sg = SliderGrid(fig[1, 1],
                (label = "frow", range = range(0, 2000, n), startvalue = 500),
                (label = "fcol", range = range(0, 2000, n), startvalue = 500),
                (label = "crow", range = range(0, 1000, n), startvalue = size(img,1) / 2),
                (label = "ccol", range = range(0, 1000, n), startvalue = size(img,2) / 2),
                (label = "k", range = range(-0.7, 0.7, n), startvalue = 0.0),
                (label = "α", range = range(-π, π, n), startvalue = 0.),
                (label = "β", range = range(-π, π, n), startvalue = 0.),
                (label = "γ", range = range(-π, π, n), startvalue = 0.),
                (label = "tx", range = range(-100, 100, n), startvalue = .0),
                (label = "ty", range = range(-100, 100, n), startvalue = .0),
                (label = "tz", range = range(-100, 100, n), startvalue = 10),
                (label = "checker_size", range = range(1, 10, n), startvalue = 1)
               )
wimg = map([slider.value for slider in sg.sliders]...) do frow, fcol, crow, ccol, k, α, β, γ, tx, ty, tz, checker_size
    intrinsic = AffineMap(SDiagonal(frow, fcol), SVector(crow, ccol))
    distort(v) = lens_distortion(v, k)
    rot = RotationVec(α, β, γ)
    tra = SVector(tx, ty, tz)
    extrinsics = AffineMap(rot, tra)
    scale = LinearMap(SDiagonal{3}(I/checker_size))
    real2image = ∘(intrinsic, distort, PerspectiveMap(), extrinsics, scale, Base.Fix2(push, 0))
    warp(img, real2image, (-100:100, -100:100))
end
ax = Axis(fig[1,2], aspect = DataAspect())
image!(ax, wimg)
