splat(f) = args -> f(args...)
push1(x) = push(x, 1.0)

function calibrationmatrices(check, extrinsic)
    path = @__DIR__()
    mat"""addpath(path,genpath($path));
    warning('off','all')"""
    mat"[$ttform, $errors] = justextrinsic($check, $extrinsic)"
    tform = ttform'
    itform = inv(tform)
    return (tform, itform, errors)
end


"""
    buildcalibration(check, extrinsic)

Build a calibration object from just a single checkerboard image (i.e. extrinsic). `check` 
is the width of the checker, and `extrinsic` is the file name of the image.
"""
function buildcalibration(check, extrinsic::AbstractString)
    _tform, _itform, ϵ = calibrationmatrices(check, extrinsic)
    M = LinearMap(SMatrix{3,3, Float64}(_tform))
    tform = PerspectiveMap() ∘ M ∘ push1
    iM = LinearMap(SMatrix{3,3, Float64}(_itform))
    itform = PerspectiveMap() ∘ iM ∘ push1
    (; tform, itform, ϵ)
end

"""
    buildcalibration(check, extrinsic, ::Missing)

A convinience function for building a calibration object from just a single checkerboard 
image (i.e. extrinsic). 
"""
buildcalibration(check, extrinsic, ::Missing) = buildcalibration(check, extrinsic)

function calibrationmatrices(check, extrinsic, intrinsic)
    path = @__DIR__()
    mat"""addpath(path,genpath($path));
    warning('off','all')"""

    mat"[$sz, cameraParams, R, t, $errors] = bothcalibrations($check, $extrinsic, $intrinsic)"

    h, w = Int.(sz)
    rows = 1:h
    cols = 1:w
    points = vcat(map.(hcat, rows, cols')...)

    mat"$world = pointsToWorld(cameraParams, R, t, $points);"
    tform = reshape(V2.(eachrow(world)), h, w)'
    _tform = (; tform, cols, rows)

    mx, Mx = extrema(world[:,1])
    my, My = extrema(world[:,2])
    xs = UnitRange(mx, Mx)
    ys = UnitRange(my, My)
    worldPoints = vcat(map.(hcat, xs, ys', 0.0)...)
    mat"$projectedPoints = worldToImage(cameraParams,R,t,$worldPoints);"
    itform = reshape(V2.(eachrow(projectedPoints)), length(xs), length(ys))
    _itform = (; itform, xs, ys)

    return (_tform, _itform, errors)
end

"""
    buildcalibration(check, extrinsic, intrinsic)

Build a calibration object from extrinsic and intrinsic images. `check` 
is the width of the checker, `extrinsic` is the file name of the image, `intrinsic` is a
collection of the intrinsic image files.
"""
function buildcalibration(check, extrinsic::AbstractString, intrinsic::AbstractVector)
    tf, itf, ϵ = calibrationmatrices(check, extrinsic, intrinsic)
    _tform = interpolate(tf.tform, BSpline(Linear()))
    tform = splat(extrapolate(scale(_tform, tf.cols, tf.rows), Flat()))
    _itform = interpolate(itf.itform, BSpline(Linear()))
    itform = splat(extrapolate(scale(_itform, itf.xs, itf.ys), Flat()))
    (; tform, itform, ϵ)
end
