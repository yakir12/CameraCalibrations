"""
    lens_distortion
Lens distortion for one radial coefficient.
"""
function lens_distortion(v, k)
    k == 0 && return v
    r² = LinearAlgebra.norm_sqr(v)
    if k < 0 && r² > -1/3/k
        throw(ArgumentError("real-world coordinates outside lens distortion model (the radial distortion of the lens is too strong)"))
    end
    radial = 1 + k*r²
    radial*v
end

intrinsic(c::Calibration) = AffineMap(SDiagonal(c.frow, c.fcol), SVector(c.crow, c.ccol))
distort(c::Calibration) = (rc) -> lens_distortion(rc, c.k)
extrinsic(c::Calibration, i::Int) = AffineMap(RotationVec(c.rotation_vecs[i]...), c.translation_vecs[i]) # check for splatting
scale(c::Calibration) = LinearMap(SDiagonal{3}(I/c.checker_size))
real2image(c::Calibration, i::Int) = ∘(intrinsic(c), distort(c), PerspectiveMap(), extrinsic(c, i), scale(c), push0)

"""
    inv_lens_distortion
Analytical inverse lens distortion for one radial coefficient.
"""
function inv_lens_distortion(v2, k) 
    r² = LinearAlgebra.norm_sqr(v2)
    if k < 0 && r² > -4/27/k
        throw(ArgumentError("image coordinates outside lens inverse-distortion model (the radial distortion of the lens is too strong)"))
    end
    c = k*r² + 0im
    term = 1 - sign(k)*sqrt(3)*1im
    radial = real((term*(2^(2/3)*term*(-27*c + sqrt((27*c + 2)^2 - 4) - 2)^(1/3) + 4)*(-27*c + sqrt((27*c + 2)^2 - 4) - 2)^(1/3) + 8*2^(1/3))/(12*term*(-27*c + sqrt((27*c + 2)^2 - 4) - 2)^(1/3)))
    v2 / radial
end
# function inv_lens_distortion(v2, k)
#     k == 0 && return v2
#     c = k*LinearAlgebra.norm_sqr(v2)
#     rs = roots(Polynomial([c, 0, 1, -1]))
#     rrs = filter(x -> abs(imag(x)) < 1e-10  , rs)
#     radial = maximum(real, rrs)
#     v2 / radial
# end

# function obj2img(k, Rs, ts, checker_size)
#     distort(rc) = lens_distortion(rc, k)
#     extrinsics = AffineMap.(Base.splat(RotationVec).(Rs), ts)
#     scale = LinearMap(SDiagonal{3}(I/checker_size))
#     return distort, extrinsics, scale
# end

# this is the inverse prespective map
depth(rc1, t, l) = -t/(l⋅rc1)
function get_inv_prespective_map(inv_extrinsic)
    function (rc)
        rc1 = push(rc, 1)
        t = inv_extrinsic.translation[3]
        l = inv_extrinsic.linear[end, :]
        d = depth(rc1, t, l)
        return d .* rc1
    end
end


inv_scale(c::Calibration) = inv(scale(c))
inv_extrinsic(c::Calibration, i::Int) = inv(extrinsic(c, i))
inv_PerspectiveMap(c::Calibration, i::Int) = get_inv_prespective_map(inv_extrinsic(c, i))
inv_distort(c::Calibration) = (rc) -> inv_lens_distortion(rc, c.k)
inv_intrinsic(c::Calibration) = inv(intrinsic(c))
image2real(c::Calibration, i::Int) = ∘(pop, inv_scale(c), inv_extrinsic(c, i), inv_PerspectiveMap(c, i), inv_distort(c), inv_intrinsic(c))



# function img2obj(extrinsics, scale, k)
#     inv_extrinsics = inv.(extrinsics)
#     inv_perspective_maps = get_inv_prespective_map.(inv_extrinsics)
#     inv_distort(rc) = inv_lens_distortion(rc, k)
#     return inv(scale), inv_extrinsics, inv_perspective_maps, inv_distort
# end



# struct Calibration
#     files # image files
#     n_files # how many
#     n_corners # the number of corners in each of the two dimensions of the checkerboard
#     checker_size # physical size of the checker (e.g. in cm)
#     objpoints # ideal real world coordinates of the checkerboard corners
#     imgpointss # the detetcted corners in each of the images, in pixel coordinates
#     k # radial lens distortion coefficient
#     real2image # the transformation that converts real world to image coordinates
#     image2real # the transformation that converts image coordinates to real world
#     sz # the dimensions of the images
# end


# invalid_distortion_coordinate(rc::RowCol, k, c) = k ≥ 0 || LinearAlgebra.norm_sqr(c.inv_intrinsic(rc)) > -4/27/k
# function invalid_distortion_coordinate(xyz::XYZ, k, c) 
#     #TODO:  maybe ditch the z dimension and keep the image plane as the extrinsic_index...? fix the get_axes function
#     to_r = PerspectiveMap() ∘ c.extrinsics[extrinsic_index] ∘ scale  ∘ Base.Fix2(push, 0)
#     k ≥ 0 || LinearAlgebra.norm_sqr(inv_intrinsic(rc)) > -4/27/k

"""
    Calibration(files, n_corners, checker_size, extrinsic_index)
Build a calibration object. `files` are the image files of the checkerboard. `n_corners` is a tuple of the number of corners in each of the sides of the checkerboard. `checker_size` is the physical size of the checker (e.g. in cm). `with_distortion` controls if radial lens distortion is included in the model or not.
"""
# function Calibration(files, n_corners, checker_size; with_distortion = true)
#     files, objpoints, imgpointss, sz, k, Rs, ts, frow, fcol, crow, ccol = detect_fit(unique(files), n_corners, with_distortion)
#     @show k, Rs, ts, frow, fcol, crow, ccol
#     intrinsic = AffineMap(SDiagonal(frow, fcol), SVector(crow, ccol))
#     inv_intrinsic = inv(intrinsic)
#     # if  k < 0
#     #     bad = find_bad_images(inv_intrinsic, files, imgpointss, k)
#     #     while !isempty(bad)
#     #         @info "some images had checkerboard corners outside the distortion model, removing bad images and rerunning calibration"
#     #         filter!(∉(bad), files)
#     #         @assert length(files) > 0 "all images had corners outside the distortion model"
#     #         files, objpoints, imgpointss, sz, k, Rs, ts, frow, fcol, crow, ccol = detect_fit(files, n_corners, with_distortion)
#     #         intrinsic = AffineMap(SDiagonal(frow, fcol), SVector(crow, ccol))
#     #         inv_intrinsic = inv(intrinsic)
#     #         bad = find_bad_images(inv_intrinsic, files, imgpointss, k)
#     #     end
#     # end
#     distort, extrinsics, scale = obj2img(k, Rs, ts, checker_size)
#     real2image = .∘(Ref(intrinsic), distort, Ref(PerspectiveMap()), extrinsics, Ref(scale))
#     inv_scale, inv_extrinsics, inv_perspective_maps, inv_distort = img2obj(extrinsics, scale, k)
#     image2real = .∘(Ref(inv_scale), inv_extrinsics, inv_perspective_maps, inv_distort, Ref(inv_intrinsic))
#
#     Calibration(files, length(files), n_corners, checker_size, objpoints, imgpointss, k, real2image, image2real, sz)
# end


"""
    c(i::RowCol, extrinsic)
Convert the row column StaticArray `i` to its real-world equivalent, `XYZ`, for the extrinsic parameters from the `extrinsic` image (given as an index or file name).
"""
(c::Calibration)(rc::RowCol, extrinsic_index::Int) = image2real(c, extrinsic_index)(rc)

"""
    c(xyz::XYZ, extrinsic)
Convert the x, y, z StaticArray `xyz` to its pixel-coordinate equivalent, `RowCol` for the extrinsic parameters from the `extrinsic` image (given as an index or file name).
"""
(c::Calibration)(xy::XY, extrinsic_index::Int) = real2image(c, extrinsic_index)(xy)

# function (c::Calibration)(coordinate, extrinsic_file::AbstractString)
#     extrinsic_index = findfirst(==(extrinsic_file), c.files)
#     c(coordinate, extrinsic_index)
# end

"""
    rectification(c, extrinsic_index)
Return a function that accepts an instance of `::RowCol` and converts it to its real-world equivalent for the extrinsic parameters from the `extrinsic_index` image, without its third dimension (which would be ≈ 0): an `xy` coordinate.
"""
# rectification(c, extrinsic_index) = pop ∘ c.image2real[extrinsic_index]

# function _reprojection(c, i)
#     imgpoints = c.imgpointss[i]
#     reprojected = c.(c.objpoints, i)
#     sum(LinearAlgebra.norm_sqr, reprojected .- imgpoints)
# end

"""
    calculate_errors(c)
Calculate reprojection, projection, distance, and inverse errors for the calibration `c`. `distance` measures the mean error of the distance between all adjacent checkerboard corners from the expected `checker_size`. `inverse` measures the mean error of applying the calibration's transformation and its inverse `inverse_samples` times.
"""
function gof(cd::CalibrationData, c::Calibration)
    reprojection = 0.0
    projection = 0.0
    distance = 0.0
    for (i, imgpoints) in pairs(cd.images_points)
        reprojection += mapreduce(+, zip(cd.object_points, imgpoints)) do (objpoint, imgpoint)
            LinearAlgebra.norm_sqr(c(objpoint, i) - imgpoint)
        end

        # projection += mapreduce(+, zip(cd.object_points, imgpoints)) do (objpoint, imgpoint)
        #     LinearAlgebra.norm_sqr(objpoint - c(imgpoint, i))
        # end

        projected = c.(imgpoints, i)
        projection += sum(LinearAlgebra.norm_sqr, projected .- cd.object_points)

        distance += sum(1:2) do dims
            sum(abs2, norm.(diff(projected; dims)) .- c.checker_size)
        end
    end
    n_files = length(cd.files)
    n = prod(cd.n_corners)*n_files
    reprojection = sqrt(reprojection/n)
    projection = sqrt(projection/n)
    distance = sqrt(distance/prod(cd.n_corners .- 1)/n_files)
    return (; reprojection, projection, distance)
end


function remove_invalid!(cd)
    c = Calibration(cd)
    c.k ≥ 0 && return cd

    bad = findall(imgpoints -> any(>(-4/27/c.k) ∘ LinearAlgebra.norm_sqr ∘ inv_intrinsic(c), imgpoints), cd.images_points)
    isempty(bad) && return cd

    deleteat!(cd.files, bad)
    deleteat!(cd.images_points, bad)
    @assert length(cd.files) > 0 "all images had corners outside the distortion model"

    remove_invalid!(cd)
end


"""
    improve
Identify all the images that had relatively high reprojection errors, and rerun the calibration without them. Include a maximum of `n` images with the lowest reprojection error, or all the images with an error lower than `threshold`.
"""
function remove_errored!(cd, threshold=2)
    c = Calibration(cd)
    bad = filter(eachindex(cd.files)) do i
        reprojection = mapreduce(+, zip(cd.object_points, cd.images_points[i])) do (objpoint, imgpoint)
            LinearAlgebra.norm_sqr(c(objpoint, i) - imgpoint)
        end
        reprojection = sqrt(reprojection/prod(cd.n_corners))
        reprojection > threshold
    end
    isempty(bad) && return cd

    deleteat!(cd.files, bad)
    deleteat!(cd.images_points, bad)
    @assert length(cd.files) > 0 "all images had reprojection errors larger than $threshold"

    remove_errored!(cd, threshold)
end
