# using Revise, CameraCalibrations
using ImageBase, ImageTransformations, Interpolations, ImageView, FileIO, StaticArrays, ImageDraw
using LinearAlgebra

function get_img(n_corners)
    ratio = 50
    corners = SVector{2, Float64}.(reverse.(Tuple.(CartesianIndices(n_corners)))) .+ Ref(SVector(0.5, 0.5))
    corners .*= ratio

    img = Matrix{Gray{N0f8}}(undef, n_corners)
    for ij in eachindex(IndexCartesian(), img)
        i, j = Tuple(ij)
        img[ij] = if isodd(i)
            isodd(j) ? Gray(1) : Gray(0)
        else
            isodd(j) ? Gray(0) : Gray(1)
        end
    end
    w, h = n_corners .+ 2
    img2 = imresize(PaddedView(Gray(1), img, (w, h), (2, 2)); ratio, method = Constant())
    return corners, img2
end
n_corners = (6, 9)
corners, img = get_img(n_corners .+ 1)
sz = size(img)
file = "img.png"

FileIO.save(file, img)

_, xys = CameraCalibrations._detect_corners(file, n_corners, sz)

imgc = RGB.(img)
for xy in xys
    draw!(imgc, Cross(Point(200,150), 50))
end

f(img, n) = kron(img, ones(Int, n, n))
g(img, n) = imresize(img; ratio = n, method = Constant())
img = collect(reshape(1.0:6.0, 2, 3))
n = 50
f(img, n) == g(img, n)

@btime f($img, $n);

@btime g($img, $n);

fun(n_corners, ratio) = kron(repeat(I(2), n_corners)[:,2:end], ones(Int, n, n))

imshow()

###########################


using Test
using GLMakie, PaddedViews, LinearAlgebra, StaticArrays

function index2bw(ij::CartesianIndex) 
    i, j = Tuple(ij)
    isodd(i) ? isodd(j) : !isodd(j)
end
function generate_checkerboard(n_corners, n) 
    xys = [ratio .* SVector{2, Float32}(Tuple(ij)) - SVector(0.5, 0.5) for ij in CartesianIndices(StepRange.(n_corners .+ 1, -1, 2))]
    img = index2bw.(CartesianIndices(n_corners .+ 1))
    imgl = kron(PaddedView(true, img, UnitRange.(0, n_corners .+ 2)), ones(Int, n, n))
    return xys, imgl
end
for _ in 1:100
    n_corners = (rand(4:15), rand(4:15))
    if isequal(isodd.(n_corners)...)
        n_corners = (n_corners[1] + 1, n_corners[2])
    end
    ratio = rand(20:100)
    xys, img = generate_checkerboard(n_corners, ratio)
    sz = size(img)
    xys2 = mktemp() do file, io
        FileIO.save(File{format"PNG"}(file), Gray.(img))
        close(io)
        _, xys2 = CameraCalibrations._detect_corners(file, n_corners, sz)
        return xys2
    end
    @test xys == xys2
end


# imgc = RGB.(img)
# for xy in xys
#     draw!(imgc, Cross(Point(xy), 5), RGB{N0f8}(1,0,0))
# end
# imshow(imgc)

fig = Figure()
ax = Axis(fig[1,1], aspect = DataAspect(), yreversed = true)
image!(ax, img, interpolate = false)
scatter!(ax, Point2f.(vec(xys)), color = :red)

using CameraCalibrations, FileIO, ColorTypes


