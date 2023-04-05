function calibrateCamera(_files, n_corners)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((1, *(n_corners...), 3), np.float32)
    for i in 1:n_corners[1], j in 1:n_corners[2]
        row = LinearIndices(n_corners)[i, j]
        objp[0][row - 1][1] = i - 1
        objp[0][row - 1][0] = j - 1
    end

    py_imgpoints = PyList([])
    py_objpoints = PyList([])
    files = String[]
    for file in _files
        img = cv2.imread(file)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, n_corners, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if Bool(ret)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
            corners3 = np.flip(corners2, axis = 2)
            push!(py_imgpoints, corners3)
            push!(py_objpoints, objp)
            push!(files, file)
        end
    end

    img = cv2.imread(files[1])
    sz = (pyconvert(Int, np.size(img, 0)), pyconvert(Int, np.size(img, 1)))

    py_deviation, py_mtx, py_dist, py_rvecs, py_tvecs = cv2.calibrateCamera(py_objpoints, py_imgpoints, np.flip(sz), nothing, nothing, flags = cv2.CALIB_ZERO_TANGENT_DIST + cv2.CALIB_FIX_K2 + cv2.CALIB_FIX_K3)

    k, _ = PyArray(py_dist)
    Rs = vec.(Matrix.(PyArray.(py_rvecs)))
    ts = vec.(Matrix.(PyArray.(py_tvecs)))
    mtx = Matrix(PyArray(py_mtx))

    frow = mtx[1,1]
    fcol = mtx[2,2]
    crow = mtx[1,3]
    ccol = mtx[2,3]

    objpoints = reshape(XYZ.(eachslice(PyArray(py_objpoints[1]); dims=2)), n_corners)
    imgpointss = [reshape(RowCol.(eachslice(PyArray(py_imgpoint); dims = 1)), n_corners) for py_imgpoint in py_imgpoints]

    return (; files, k, Rs, ts, frow, fcol, crow, ccol, objpoints, imgpointss, sz)
end

function lens_distortion(v, k)
    r² = LinearAlgebra.norm_sqr(v)
    radial = 1 + k*r²
    radial*v
end

function inv_lens_distortion(v2, k)
    c = k*LinearAlgebra.norm_sqr(v2)
    rs = roots(Polynomial([c, 0, 1, -1]))
    rrs = filter(x -> abs(imag(x)) < 1e-10  , rs)
    radial = maximum(real, rrs)
    v2 / radial
end

function obj2img(frow, fcol, crow, ccol, k, Rs, ts, checker_size)
    intrinsic = AffineMap(SDiagonal(frow, fcol), SVector(crow, ccol))
    distort(rc) = lens_distortion(rc, k)
    # extrinsics = Dict(file => AffineMap(RotationVec(Rs[file]...), ts[file]) for file in keys(Rs))
    extrinsics = AffineMap.(Base.splat(RotationVec).(Rs), ts)
    scale = LinearMap(SDiagonal{3}(I/checker_size))
    return intrinsic, distort, extrinsics, scale
end

depth(rc1, t, l) = -t/(l⋅rc1)
function get_prespective_map(inv_extrinsic)
    function (rc)
        rc1 = push(rc, 1)
        d = depth(rc1, inv_extrinsic.translation[3], inv_extrinsic.linear[end, :])
        return d .* rc1
    end
end

function img2obj(scale, extrinsics, intrinsic, k)
    inv_extrinsics = inv.(extrinsics)
    # inv_extrinsics = Dict(file => inv(extrinsic) for (file, extrinsic) in extrinsics)
    inv_perspective_maps = get_prespective_map.(inv_extrinsics)
    # inv_perspective_maps = Dict(file => get_prespective_map(inv_extrinsic) for (file, inv_extrinsic) in inv_extrinsics)
    inv_intrinsic = inv(intrinsic)
    inv_distort(rc) = inv_lens_distortion(rc, k)
    return inv(scale), inv_extrinsics, inv_perspective_maps, inv_distort, inv_intrinsic
end

struct Calibration
    files
    n_corners
    checker_size
    objpoints
    imgpointss
    real2image
    image2real
    sz
end

"""
    Calibration(files, n_corners, checker_size, extrinsic_index)
Build a calibration object. `files` are the image files of the checkerboard. `n_corners` is a tuple of the number of corners in each of the sides of the checkerboard. `checker_size` is the physical size of the checker (e.g. in cm). 
"""
function Calibration(files, n_corners, checker_size)
    files, k, Rs, ts, frow, fcol, crow, ccol, objpoints, imgpointss, sz = calibrateCamera(unique(files), n_corners)
    objpoints .*= checker_size
    intrinsic, distort, extrinsics, scale = obj2img(frow, fcol, crow, ccol, k, Rs, ts, checker_size)
    real2image = .∘(Ref(intrinsic), distort, Ref(PerspectiveMap()), extrinsics, Ref(scale))
    inv_scale, inv_extrinsics, inv_perspective_maps, inv_distort, inv_intrinsic = img2obj(scale, extrinsics, intrinsic, k)
    image2real = .∘(Ref(inv_scale), inv_extrinsics, inv_perspective_maps, inv_distort, Ref(inv_intrinsic))
    Calibration(files, n_corners, checker_size, objpoints, imgpointss, real2image, image2real, sz)
end

"""
    c(i::RowCol, extrinsic)
Convert the row column StaticArray `i` to its real-world equivalent, `XYZ`, for the extrinsic parameters from the `extrinsic` image (given as an index or file name).
"""
(c::Calibration)(i::RowCol, extrinsic_index::Int) = c.image2real[extrinsic_index](i)

"""
    c(xyz::XYZ, extrinsic)
Convert the x, y, z StaticArray `xyz` to its pixel-coordinate equivalent, `RowCol` for the extrinsic parameters from the `extrinsic` image (given as an index or file name).
"""
(c::Calibration)(xyz::XYZ, extrinsic_index::Int) = c.real2image[extrinsic_index](xyz)

function (c::Calibration)(coordinate, extrinsic_file::AbstractString)
    extrinsic_index = findfirst(==(extrinsic_file), c.files)
    c(coordinate, extrinsic_index)
end

"""
    rectification(c, extrinsic_index)
Return a function that accepts an instance of `::RowCol` and converts it to its real-world equivalent for the extrinsic parameters from the `extrinsic_index` image, without its third dimension (which would be ≈ 0): an `xy` coordinate.
"""
rectification(c, extrinsic_index) = pop ∘ c.image2real[extrinsic_index]

"""
    calculate_errors(c)
Calculate reprojection, projection, distance, and inverse errors for the calibration `c`. `distance` measures the mean error of the distance between all adjacent checkerboard corners from the expected `checker_size`. `inverse` measures the mean error of applying the calibration's transformation and its inverse `inverse_samples` times.
"""
function calculate_errors(c, inverse_samples=1000)
    reprojection = 0.0
    projection = 0.0
    distance = 0.0
    inverse = 0.0
    for (i, imgpoints) in pairs(c.imgpointss)
        reprojected = c.(c.objpoints, i)
        reprojection += sum(LinearAlgebra.norm_sqr, reprojected .- imgpoints)

        projected = c.(imgpoints, i)
        projection += sum(LinearAlgebra.norm_sqr, projected .- c.objpoints)

        distance += sum(1:2) do dims
            sum(abs2, norm.(diff(projected; dims)) .- c.checker_size)
        end

        inverse += sum(1:inverse_samples) do _
            rc = rand(RowCol{Float64}) .* (c.sz .- 1) .+ 1
            projected = c(rc, i)
            reprojected = c(projected, i)
            LinearAlgebra.norm_sqr(rc .- reprojected)
        end
    end
    nfiles = length(c.files)
    n = prod(c.n_corners)*nfiles
    reprojection = sqrt(reprojection/n)
    projection = sqrt(projection/n)
    distance = sqrt(distance/prod(c.n_corners .- 1)/nfiles)
    inverse = sqrt(inverse/inverse_samples/nfiles)
    return (; reprojection, projection, distance, inverse)
end

