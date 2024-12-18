"""
    _detect_corners
Wraps OpenCV function to auto-detect corners in an image.
"""
function _detect_corners(file, n_corners, sz)
    img = FileIO.load(file)
    gry = reshape(rawview(channelview(Gray.(img))), 1, sz...)
    cv_n_corners = OpenCV.Size{Int32}(n_corners...)
    _cv_corners = OpenCV.Mat(Array{Float32}(undef, 2, 1, prod(n_corners)))
    ret, cv_corners = OpenCV.findChessboardCorners(gry, cv_n_corners, _cv_corners, 
                                                     OpenCV.CALIB_CB_ADAPTIVE_THRESH +
                                                     OpenCV.CALIB_CB_FAST_CHECK +
                                                     OpenCV.CALIB_CB_EXHAUSTIVE + # Run an exhaustive search to improve detection rate.
                                                     OpenCV.CALIB_CB_ACCURACY # Up sample input image to improve sub-pixel accuracy due to aliasing effects.
                                                    )
    !ret && return missing
    ref_corners = OpenCV.cornerSubPix(gry, cv_corners, OpenCV.Size{Int32}(11,11), OpenCV.Size{Int32}(-1,-1), CRITERIA)
    corners = reshape(RowCol.(eachslice(ref_corners, dims = 3)), n_corners)
    return (file, corners)
end

"""
    fit_model
Wraps OpenCV function to fit a camera model to given object and image points.
"""
function fit_model(sz, objpoints, imgpointss, n_corners,  with_distortion, aspect)
    nfiles = length(imgpointss)

    objectPoints = OpenCV.InputArray[Float32.(reshape(stack(objpoints), 3, 1, :)) for _ in 1:nfiles]
    imagePoints = OpenCV.InputArray[Float32.(reshape(stack(imgpoints), 2, 1, :)) for imgpoints in imgpointss]
    imageSize = OpenCV.Size{Int32}(sz...)

    cammat = collect(I(3))
    cammat[1,:] .= aspect
    cameraMatrix = OpenCV.Mat(Float32.(reshape(cammat, 1, 3, 3)))
    distCoeffs = OpenCV.Mat(Array{Float32}(undef, 1, 1, 5))

    flags = OpenCV.CALIB_ZERO_TANGENT_DIST + OpenCV.CALIB_FIX_K3 + OpenCV.CALIB_FIX_K2 + (with_distortion ? 0 : OpenCV.CALIB_FIX_K1) + OpenCV.CALIB_FIX_ASPECT_RATIO

    r = OpenCV.InputArray[Array{Float32}(undef, 3, 1, prod(n_corners)) for _ in 1:nfiles]
    t = OpenCV.InputArray[Array{Float32}(undef, 3, 1, prod(n_corners)) for _ in 1:nfiles]

    x, mtx, dist, rvecs, tvecs = OpenCV.calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, r, t, flags, CRITERIA)

    k, _ = dist

    Rs = vec.(rvecs)
    ts = vec.(tvecs)
    # Rs = reverse.(vec.(rvecs))
    # ts = reverse.(vec.(tvecs))

    mtx = reshape(mtx, 3, 3)'
    frow = mtx[1,1]
    fcol = mtx[2,2]
    crow = mtx[1,3]
    ccol = mtx[2,3]
    # fcol = mtx[1,1]
    # frow = mtx[2,2]
    # ccol = mtx[1,3]
    # crow = mtx[2,3]

    return (; k, Rs, ts, frow, fcol, crow, ccol)
end

function detect_fit(_files, n_corners, with_distortion, aspect)
    sz = size(FileIO.load(first(_files)))
    fi = skipmissing(_detect_corners.(_files, Ref(n_corners), Ref(sz)))
    @assert !isempty(fi) "No checkers were detected in any of the images, perhaps try a different `n_corners`."
    files = first.(fi)
    imgpointss = last.(fi)
    # objpoints = XYZ.(Tuple.(CartesianIndices(((n_corners[1] - 1):-1:0, (n_corners[2] - 1):-1:0, 0:0))))
    objpoints = XYZ.(Tuple.(CartesianIndices((0:(n_corners[1] - 1), 0:(n_corners[2] - 1), 0:0))))
    # objpoints = XYZ.(Tuple.(CartesianIndices(((0:n_corners[1] - 1), (n_corners[2] - 1):-1:0, 0:0))))
    k, Rs, ts, frow, fcol, crow, ccol = fit_model(sz, objpoints, imgpointss, n_corners, with_distortion, aspect)
    return (; files, objpoints, imgpointss, sz, k, Rs, ts, frow, fcol, crow, ccol)
end
