"""
    radial
Radial lens distortion.
"""
radial(r², k₁, k₂, k₃) = 1 + k₁*r² + k₂*r²^2 + k₃*r²^3

"""
    tangential
Tangential lens distortion.
"""
tangential(r², v, p₁, p₂) = SV(2p₁*prod(v) + p₂*(r² + 2v.x^2), 2p₂*prod(v) + p₁*(r² + 2v.y^2))

"""
    lens_distortion
Radial and tangential Lens distortion.
"""
function lens_distortion(v, k₁, k₂, k₃, p₁, p₂)
    r² = LinearAlgebra.norm_sqr(v)
    rad = radial(r², k₁, k₂, k₃)
    tang = tangential(r², v, p₁, p₂)
    tang + rad*v
end

intrinsic(cam::Camera) = AffineMap(SDiagonal(cam.focal_length...), cam.principal_point)
distort(cam::Camera) = (rc) -> lens_distortion(rc, cam.distortion...)
extrinsic(cam::Camera, i::Int) = AffineMap(RotationVec(cam.rotation_vecs[i]...), cam.translation_vecs[i])
scale(cam::Camera) = LinearMap(SDiagonal{3}(I*cam.pixel_size))

"""
    real2image

Return a function that converts a 2D real-world coordinate (the z dimension is assumed to be zero) to an image-pixel coordinate. `i` denotes the extrinsic image you want to convert the point to.
"""
real2image(cam::Camera, i::Int) = ∘(intrinsic(cam), distort(cam), PerspectiveMap(), extrinsic(cam, i), scale(cam), push0)

"""
    inv_lens_distortion
Analytical inverse lens distortion for one radial coefficient.
"""
function inv_lens_distortion(v, k₁, k₂, k₃, p₁, p₂, n = 20)
    v0 = copy(v)
    for j = 1:n
        r² = LinearAlgebra.norm_sqr(v)
        rad = radial(r², k₁, k₂, k₃)
        if rad < 0
            break
        end
        tang = tangential(r², v, p₁, p₂)
        v = (v0 - tang)/rad
    end
    return v
end


# Inverse Perspective map
depth(rc1, t, l) = -t/(l⋅rc1)
function get_inv_perspective_map(inv_extrinsic)
    function (rc)
        rc1 = push(rc, 1)
        t = inv_extrinsic.translation[3]
        l = inv_extrinsic.linear[end, :]
        d = depth(rc1, t, l)
        return d .* rc1
    end
end

inv_scale(cam::Camera) = inv(scale(cam))
inv_extrinsic(cam::Camera, i::Int) = inv(extrinsic(cam, i))
inv_PerspectiveMap(cam::Camera, i::Int) = get_inv_perspective_map(inv_extrinsic(cam, i))
inv_distort(cam::Camera) = (rc) -> inv_lens_distortion(rc, cam.distortion...)
inv_intrinsic(cam::Camera) = inv(intrinsic(cam))

"""
    image2real

Return a function that converts an image-pixel coordinate to a 2D real-world coordinate (the z dimension is assumed to be zero). `i` denotes the extrinsic image you want to convert the point from.
"""
image2real(cam::Camera, i::Int) = ∘(pop, inv_scale(cam), inv_extrinsic(cam, i), inv_PerspectiveMap(cam, i), inv_distort(cam), inv_intrinsic(cam))


"""
    gof(calibration, camera)
Calculate reprojection, projection, and distance errors for the calibration and camera. `distance` measures the mean error of the distance between all adjacent checkerboard corners from the expected `checker_size`.
"""
function gof(calib::Calibration, cam::Camera)
    reprojection = 0.0
    projection = 0.0
    distance = 0.0
    for (i, imgpoints) in pairs(calib.images_points)
        reprojection += mapreduce(+, zip(calib.object_points, imgpoints)) do (objpoint, imgpoint)
            LinearAlgebra.norm_sqr(real2image(cam, i)(objpoint) - imgpoint)
        end
        projected = image2real(cam, i).(imgpoints)
        projection += sum(LinearAlgebra.norm_sqr, projected .- calib.object_points)

        distance += sum(1:2) do dims
            sum(abs2, norm.(diff(projected; dims)) .- calib.checker_size)
        end
    end
    n_files = length(calib.files)
    n = prod(calib.n_corners)*n_files
    reprojection = sqrt(reprojection/n)
    projection = sqrt(projection/n)
    distance = sqrt(distance/prod(calib.n_corners .- 1)/n_files)
    return (; reprojection, projection, distance)
end

"""
    remove_errored!
Identify all the images that had reprojection errors higher than `threshold`, and remove them from the Calibration instance.
"""
function remove_errored!(calib::Calibration, threshold=2)
    cam = Camera(calib)
    bad = filter(eachindex(calib.files)) do i
        reprojection = mapreduce(+, zip(calib.object_points, calib.images_points[i])) do (objpoint, imgpoint)
            LinearAlgebra.norm_sqr(real2image(cam, i)(objpoint) - imgpoint)
        end
        reprojection = sqrt(reprojection/prod(calib.n_corners))
        reprojection > threshold
    end
    isempty(bad) && return calib

    deleteat!(calib.files, bad)
    deleteat!(calib.images_points, bad)
    @assert length(calib.files) > 0 "all images had reprojection errors larger than $threshold"

    remove_errored!(calib, threshold)
end
