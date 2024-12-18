using Revise, CameraCalibrations
using ImageBase, ImageTransformations, Interpolations, ImageView, FileIO, StaticArrays, ImageDraw

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
    draw!(imgc, Cross(Point(200,150), 50),


