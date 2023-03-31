reverse_ab(abc) = XYZ(abc[2], abc[1], abc[3])

function calibrateCamera(_files, n_corners, extrinsic_file)
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

    extrinsic_index = findfirst(==(extrinsic_file), files)
    @assert !isnothing(extrinsic_index) "failed to detect checkerboard in the extrinsic image file: $extrinsic_file"
    img = cv2.imread(files[1])
    sz = (pyconvert(Int, np.size(img, 0)), pyconvert(Int, np.size(img, 1)))

    py_deviation, py_mtx, py_dist, py_rvecs, py_tvecs = cv2.calibrateCamera(py_objpoints, py_imgpoints, np.flip(sz), nothing, nothing, flags = cv2.CALIB_ZERO_TANGENT_DIST + cv2.CALIB_FIX_K2 + cv2.CALIB_FIX_K3)

    deviation = pyconvert(Float64, py_deviation) # deviation should be less than 1

    k, _ = PyArray(py_dist)
    Rs = vec.(Matrix.(PyArray.(py_rvecs)))
    ts = vec.(Matrix.(PyArray.(py_tvecs)))
    mtx = Matrix(PyArray(py_mtx))

    frow = mtx[1,1]
    fcol = mtx[2,2]
    crow = mtx[1,3]
    ccol = mtx[2,3]

    objpoints = reshape(XYZ.(reverse_ab.(eachslice(PyArray(py_objpoints[1]); dims=2))), n_corners)
    imgpointss = [reshape(RowCol.(eachslice(PyArray(py_imgpoint); dims = 1)), n_corners) for py_imgpoint in py_imgpoints]

    return (; k, Rs, ts, frow, fcol, crow, ccol, objpoints, imgpointss, files, extrinsic_index, sz, deviation)
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

function obj2img(frow, fcol, crow, ccol, k, R, t, checker_size)
    intrinsic = AffineMap(SDiagonal(frow, fcol), SVector(crow, ccol))
    distort(rc) = lens_distortion(rc, k)
    extrinsic = AffineMap(RotationVec(R...), t)
    scale = LinearMap(SDiagonal{3}(I/checker_size))
    return intrinsic, distort, extrinsic, scale
end

depth(rc1, t, l) = -t/(l⋅rc1)
function img2obj(scale, extrinsic, intrinsic, k)
    inv_extrinsic = inv(extrinsic)
    inv_perspective_map = function (rc)
        rc1 = push(rc, 1)
        d = depth(rc1, inv_extrinsic.translation[3], inv_extrinsic.linear[end, :])
        return d .* rc1
    end
    inv_intrinsic = inv(intrinsic)
    inv_distort(rc) = inv_lens_distortion(rc, k)
    return inv(scale), inv_extrinsic, inv_perspective_map, inv_distort, inv_intrinsic
end


struct Calibration
    files
    n_corners
    checker_size

    intrinsic
    distort
    extrinsic
    scale
    inv_perspective_map
    inv_distort

    sz
    objpoints
    imgpointss
    extrinsic_index

    deviation
end

"""
    Calibration(files, n_corners, checker_size, extrinsic_index)
Build a calibration object. `files` are the image files of the checkerboard. `n_corners` is a tuple of the number of corners in each of the sides of the checkerboard. `checker_size` is the physical size of the checker (e.g. in cm). `extrinsic_file` the file that denotes the extrinsic image of the calibration.
"""
function Calibration(files, n_corners, checker_size, extrinsic_file)
    @assert allunique(files) "files should be unique"
    @assert extrinsic_file ∈ files "extrinsic file should be included in the files"
    k, Rs, ts, frow, fcol, crow, ccol, objpoints, imgpointss, files, extrinsic_index, sz, deviation = calibrateCamera(files, n_corners, extrinsic_file)
    R, t = (Rs[extrinsic_index], ts[extrinsic_index])
    intrinsic, distort, extrinsic, scale = obj2img(frow, fcol, crow, ccol, k, R, t, checker_size)
    inv_scale, inv_extrinsic, inv_perspective_map, inv_distort, inv_intrinsic = img2obj(scale, extrinsic, intrinsic, k)
    Calibration(files, n_corners, checker_size, intrinsic, distort, extrinsic, scale, inv_perspective_map, inv_distort, sz, objpoints, imgpointss, extrinsic_index, deviation)
end

function (c::Calibration)(i::RowCol)
    image2real = inv(c.scale) ∘ inv(c.extrinsic) ∘ c.inv_perspective_map ∘ c.inv_distort ∘ inv(c.intrinsic)
    image2real(i)
end

function (c::Calibration)(xyz::XYZ)
    real2image = c.intrinsic ∘ c.distort ∘ PerspectiveMap() ∘ c.extrinsic ∘ c.scale
    real2image(xyz)
end

function calculate_errors(c)
    imgpoints = c.imgpointss[c.extrinsic_index]
    scaled_objpoints = reverse_ab.(c.objpoints .* c.checker_size)
    projected_image = mean(norm, c.(scaled_objpoints) .- imgpoints)

    projected_real = c.(imgpoints)
    project_real = mean(norm, projected_real .- scaled_objpoints)

    inverse = mean(1:1000) do _
        i = rand(RowCol{Float64}) .* (c.sz .- 1) .+ 1
        norm(c(c(i)) - i)
    end

    distortion = mean(1:1000) do _
        uv1 = rand(RowCol{Float64}) .* (c.sz .- 1) .+ 1
        rc = inv(c.intrinsic)(uv1)
        norm(c.distort(c.inv_distort(rc)) - rc)
    end

    l = mean(1:2) do dims
        mean(norm, diff(projected_real; dims))
    end
    distance = abs(l - c.checker_size)

    return (; projected_image, project_real, inverse, distortion, distance, model = c.deviation)
end

function get_axes(ratio, checker_size, n_corners, sz)
    w = round.(Int, ratio .* checker_size .* (n_corners .- 1))
    min_col, min_row = round.(Int, (w .- sz) ./ 2)
    max_col, max_row = (min_col, min_row) .+ sz .- 1
    return min_row:max_row, min_col:max_col
end

function get_ratio(imgpoints, checker_size)
    l = mean(1:2) do dims
        mean(norm, diff(imgpoints; dims))
    end
    l/checker_size
end

function rectify(img, c; ratio=get_ratio(c.imgpointss[c.extrinsic_index], c.checker_size))
    s = LinearMap(SDiagonal{2}(ratio*I))
    tform =  c ∘ Base.Fix2(push, 0) ∘ inv(s)
    axs = get_axes(ratio, c.checker_size, c.n_corners, c.sz)
    warp(img, tform, axs)
end

