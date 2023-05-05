using Revise, CameraCalibrations


n_corners = (8, 5)
dir = "../test/example"
files = readdir(dir; join=true)
checker_size = 1
lens_distortion = [zero_k₁, zero_k₂, zero_k₃, zero_tangential]
calib = Calibration(files, n_corners, checker_size; lens_distortion)
cam = Camera(calib)
plot(calib, cam)
gof(calib, cam)

# remove_errored!(calib, 0.1)
# calib.files
# cam = Camera(calib)
# gof(calib, cam)

# using LinearAlgebra, Statistics
# mean(norm, real2image(cam, 1).(calib.object_points) .- calib.images_points[1])
# mean(norm, image2real(cam, 1).(calib.images_points[1]) .- calib.object_points)




# using ImageView

using Statistics, LinearAlgebra
using FileIO, StaticArrays, CoordinateTransformations, Rotations, PaddedViews
using ImageTransformations, Interpolations, Colors
using GLMakie

tf = isodd.(sum.(Tuple.(CartesianIndices(n_corners .+ 1))))
img0 = repeat(tf, inner = (10, 10));
w = 10
img = collect(PaddedView(true, img0, size(img0) .+ 2w, (w, w) .+ 1));
fig = Figure()
ax = Axis(fig[1,1], yreversed = true,  aspect = DataAspect())
image!(ax, img, interpolate = false)
#
#
f = calib.image_size[2] / calib.n_corners[1] / 3
ratio = f/calib.checker_size
s = LinearMap(SDiagonal{2}(I/ratio))
ws = round.(Int, f .* (reverse(calib.n_corners) .+ 1))
axs = Tuple((-w:sz-w-1 for (w,sz) in zip(ws, calib.image_size)))
extrinsic_index = 1
tform = inv(s) ∘ image2real(cam, extrinsic_index) ∘ SV
itform = real2image(cam, extrinsic_index) ∘ s ∘ SV
fig = Figure()
imgw = warp(img, tform, axs; method=BSpline(Constant()));
ax = Axis(fig[1,1], yreversed = true,  aspect = DataAspect())
image!(ax, imgw, interpolate=false)



xmin = typemax(Int)
xmax = typemin(Int)
ymin = typemax(Int)
ymax = typemin(Int)
for extrinsic_index in 1:length(c1.rotation_vecs)
    xs, ys = ImageTransformations.autorange(img, reverse ∘ CameraCalibrations.real2image(c1, extrinsic_index))
    xmin = min(xmin, first(xs))
    xmax = max(xmax, last(xs))
    ymin = min(ymin, first(ys))
    ymax = max(ymax, last(ys))
end
axs = (xmin:xmax, ymin:ymax)
mkpath("data")
for extrinsic_index in 1:length(c1.rotation_vecs)
    imgw = warp(img, CameraCalibrations.image2real(c1, extrinsic_index) ∘ reverse, axs; method=BSpline(Constant()));
    save(joinpath("data", "$extrinsic_index.png"), parent(imgw)')
end

dir = "data"
files = readdir(dir; join=true)
cd2 = CalibrationData(files, n_corners, checker_size, lens_distortion)
c2 = Calibration(cd2)
CameraCalibrations.plot(cd2, c2)

isapprox(c1.rotation_vecs, c2.rotation_vecs, atol = 0.01)
isapprox(c1.translation_vecs, c2.translation_vecs, atol = 0.1)



using Colors, ImageIO, GLMakie, CoordinateTransformations, Interpolations
img = RGB.(load("5.png"))
w, h = size(img)
coords = rand(CartesianIndices((w, h)), 10)
img[coords] .= RGB(1, 0, 0)
image(1:w, 1:h, img, interpolate = false, axis = (; aspect = DataAspect())) # image of h × w matrix is plotted x=1:h and y=1:w, and is displayed 90° counter-clock-wise
tcoords = Tuple.(coords)
scatter!(tcoords; marker = '+') # cartesian indices are the correct coordinates to plot onto the image matrix, no need for reverse

tform = AffineMap(SMatrix{2,2}([2 -1; 1 3]), SVector(100, -200))
tcoords2 = tform.(tcoords) # transformation works on the Cartesian Indices
imgw = warp(img, inv(tform); method=BSpline(Constant()));
image(imgw, interpolate = false, axis = (; aspect = DataAspect())) # this works because imgw has offset axes
scatter!(tcoords2; marker = '+')
save("5w.png", imgw) # saved with correct orientation

img2 = load("5w.png")
xs, ys = UnitRange.(axes(imgw)) # the axes of the warped image are not necessarily the tform of (1,1) and (w,h). It is possible that the other corners of the image are the one dictating the maximal size of the warped image.
image(xs, ys, img2, interpolate = false, axis = (; aspect = DataAspect()))
scatter!(tcoords2; marker = '+')




checker_size = 40
c1 = Calibration(checker_size, (k₁ = 0.0, k₂ = 0.0, k₃ = 0.0, p₁ = 0.0, p₂ = 0.0), [SVector{3, Float64}(π/5, deg2rad(-5), 0)], [SVector{3, Float64}(-2,-1,8)], 400, 400, 180, 250)
n_corners = (5, 8)
tf = isodd.(sum.(Tuple.(CartesianIndices(n_corners .+ 1))))
img0 = repeat(tf, inner = (50, 50));
w = 25
img = RGB.(PaddedView(true, img0, size(img0) .+ 2w, (w, w) .+ 1));
image_points = 50*XY.(tuple.(1:1:n_corners[1], (1:1:n_corners[2])')) .+ Ref(XY(25, 25))
# img2 = CameraCalibrations.draw_crosses!(img, image_points, n_corners[1], colorant"red");
# save(joinpath("data", "0.png"), img2)

fig = Figure()
ax = Axis(fig[1,1], yreversed = true,  aspect = DataAspect())
image!(ax, img)
scatter!(ax, vec(image_points), marker = '+')

image_points_w = real2image(c1, 1).(image_points)
object_points = map(xy -> checker_size*(xy .- 1), XY.(Tuple.(CartesianIndices(n_corners))))

sz = (ceil(Int, maximum(last, image_points_w)), ceil(Int, maximum(first, image_points_w)))

cd1 = CalibrationData([""], n_corners, checker_size, [image_points_w], object_points, sz, lens_distortion)

c2 = Calibration(cd1)

imgw = warp(img, reverse ∘ CameraCalibrations.image2real(c1, extrinsic_index), axs; method=BSpline(Constant()));



mkpath("data")
# save(joinpath("data", "0.png"), img)
xmin = typemax(Int)
xmax = typemin(Int)
ymin = typemax(Int)
ymax = typemin(Int)
for extrinsic_index in 1:length(c1.rotation_vecs)
    xs, ys = ImageTransformations.autorange(img, CameraCalibrations.real2image(c1, extrinsic_index) ∘ reverse)
    xmin = min(xmin, first(xs))
    xmax = max(xmax, last(xs))
    ymin = min(ymin, first(ys))
    ymax = max(ymax, last(ys))
end
axs = (xmin:xmax, ymin:ymax)
length.(axs)
for extrinsic_index in 1:length(c1.rotation_vecs)
    imgw = warp(img, reverse ∘ CameraCalibrations.image2real(c1, extrinsic_index), axs; method=BSpline(Constant()));
    save(joinpath("data", "$extrinsic_index.png"), parent(imgw))
end
dir = "data"
files = readdir(dir; join=true)
lens_distortion = [zero_k₁, zero_k₂, zero_k₃, zero_tangential]
cd1 = CalibrationData(files, n_corners, checker_size, lens_distortion)

gof(cd1, c1)

dir = "data"
files = readdir(dir; join=true)
lens_distortion = [zero_k₁, zero_k₂, zero_k₃, zero_tangential]
cd2 = CalibrationData(files, n_corners, checker_size, lens_distortion)
c2 = Calibration(cd2)
CameraCalibrations.plot(cd2, c2)

gof(cd2, c2)







fig = Figure()
ax = Axis3(fig[1,1], aspect=:data, elevation = pi/6, azimuth = pi/2)
image!(ax, img; interpolate=false)

n = 1001
sg = SliderGrid(fig[1, 1],
                (label = "frow", range = range(0, 2000, n), startvalue = 500),
                (label = "fcol", range = range(0, 2000, n), startvalue = 500),
                (label = "crow", range = range(0, 1000, n), startvalue = size(img,1) / 2),
                (label = "ccol", range = range(0, 1000, n), startvalue = size(img,2) / 2),
                (label = "sx", range = range(-π, π, n), startvalue = 0.),
                (label = "sy", range = range(-π, π, n), startvalue = 0.),
                (label = "sz", range = range(-π, π, n), startvalue = 0.),
                (label = "tx", range = range(-100, 100, n), startvalue = .0),
                (label = "ty", range = range(-100, 100, n), startvalue = .0),
                (label = "tz", range = range(-100, 100, n), startvalue = 10),
                (label = "checker_size", range = range(1, 10, n), startvalue = 1)
               )
wimg = map([slider.value for slider in sg.sliders]...) do frow, fcol, crow, ccol, sx, sy, sz, tx, ty, tz, checker_size
    c = Calibration(checker_size, c.lens_distortion, SVector(sx, sy, sz), 

                    checker_size::Float64
                    lens_distortion::NamedTuple
                    rotation_vecs::Vector{SVector{3, Float64}}
                    translation_vecs::Vector{SVector{3, Float64}}
                    frow::Float64
                    fcol::Float64
                    crow::Float64
                    ccol::Float64
                    intrinsic = AffineMap(SDiagonal(frow, fcol), SVector(crow, ccol))
                    distort(v) = CameraCalibrations.lens_distortion(v, c.lens_distortion...)
                    rot = RotationVec(sx, sy, sz)
                    tra = SVector(tx, ty, tz)
                    extrinsics = AffineMap(rot, tra)
                    scale = LinearMap(SDiagonal{3}(I/checker_size))
                    real2image = ∘(intrinsic, distort, PerspectiveMap(), extrinsics, scale, Base.Fix2(push, 0))
                    warp(img, real2image, (-100:100, -100:100))
                end
                ax = Axis(fig[1,2], aspect = DataAspect())
                image!(ax, wimg)
