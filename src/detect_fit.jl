# the following convertion functions are necessary due to some overly eager type checking in the python functions.
convert_from_py_corners(py_corners, n_corners) = reshape(RowCol.(eachslice(PyArray(py_corners); dims = 1)), n_corners)

# convert_to_py_objpoints(objpoints, n) = PyList(fill(np.array(Float32.(reshape(reduce((x1, x2) -> hcat(x1, Vector(x2)), objpoints)', (1, length(objpoints), 3)))), n))

function convert_to_py_objpoints(objpoints, n)
    lo = length(objpoints)
    m = Float32.(reshape(reduce((x1, x2) -> hcat(x1, Vector(x2)), objpoints)', (1, lo, 2)))
    m0 = cat(m, zeros(Float32, 1, lo, 1), dims=3)
    PyList(fill(np.array(m0), n))
end

convert_to_py_imgpointss(imgpointss) = PyList([np.array(reshape(reduce((x1, x2) -> hcat(x1, Vector(x2)), imgpoints)', 1, length(imgpoints), 2)) for imgpoints in imgpointss])

# const XYZ = SVector{3, <: Real}

# """
#     get_object_points
# Produce the real-world locations of the corners of the checkerboard.
# """
# function get_object_points(n_corners, checker_size)
#     objpoints = Matrix{XYZ}(undef, n_corners)
#     for i in CartesianIndices(n_corners)
#         x, y = (Tuple(i) .- 1) .* checker_size
#         objpoints[i] = XYZ(x, y, 0)
#     end
#     return objpoints
# end

"""
    _detect_corners
Wraps OpenCV function to auto-detect corners in an image.
"""
function _detect_corners(file, n_corners)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    img = cv2.imread(file)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, py_corners = cv2.findChessboardCorners(gray, n_corners, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK)

    !Bool(ret) && return missing

    ref_corners = cv2.cornerSubPix(gray, py_corners, (11,11),(-1,-1), criteria)
    flp_corners = np.flip(ref_corners, axis = 2)
    corners = convert_from_py_corners(flp_corners, n_corners)
    return (file, corners)
end

function detect_corners(_files, n_corners)
    fi = skipmissing(_detect_corners.(_files, Ref(n_corners)))
    @assert !isempty(fi) "No corners were detected in any of the image files"
    return (first.(fi), last.(fi))
end

"""
    fit_model
Wraps OpenCV function to fit a camera model to given object and image points.
"""



function CalibrationData(_files, n_corners, checker_size, with_distortion)
    files, images_points = detect_corners(_files, n_corners)
    object_points = map(xy -> checker_size*(xy .- 1), XY.(Tuple.(CartesianIndices(n_corners))))
    CalibrationData(files, n_corners, checker_size, images_points, object_points, size(load(files[1])), with_distortion)
end

function Calibration(cd::CalibrationData)
    flags = cv2.CALIB_ZERO_TANGENT_DIST + cv2.CALIB_FIX_K3 + cv2.CALIB_FIX_K2 + (cd.with_distortion ? 0 : cv2.CALIB_FIX_K1)

    _, py_mtx, py_dist, py_rvecs, py_tvecs = cv2.calibrateCamera(convert_to_py_objpoints(cd.object_points ./ cd.checker_size, length(cd.files)), convert_to_py_imgpointss(cd.images_points), np.flip(cd.image_size), nothing, nothing; flags)

    k, _ = PyArray(py_dist)
    @assert cd.with_distortion || k == 0 "distortion was $(cd.with_distortion) but k isn't zero:" k

    Rs = SVector{3, Float64}.(Matrix.(PyArray.(py_rvecs)))

    ts = SVector{3, Float64}.(Matrix.(PyArray.(py_tvecs)))

    mtx = Matrix(PyArray(py_mtx))
    frow = mtx[1,1]
    fcol = mtx[2,2]
    crow = mtx[1,3]
    ccol = mtx[2,3]

    Calibration(cd.checker_size, k, Rs, ts, frow, fcol, crow, ccol)
end

