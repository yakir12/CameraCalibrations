splat(f) = args -> f(args...)

"""
    buildcalibration(check, extrinsic, ::Missing)

Build a calibration object from just a single checkerboard image (i.e. extrinsic). `check` 
is the width of the checker, and `extrinsic` is the file name of the image.
"""
function buildcalibration(check, extrinsic::AbstractString)
    mat"""
    warning('off','all')
    I = imread($extrinsic);
    [imagePoints, boardSize] = detectCheckerboardPoints(I);
    $failed = prod(boardSize) <= 1;
    worldPoints = generateCheckerboardPoints(boardSize, $check);
    tform_ = fitgeotrans(imagePoints, worldPoints, 'projective');
    $tform = tform_.T;
    [x, y] = transformPointsForward(tform_, imagePoints(:,1), imagePoints(:,2));
    $errors = vecnorm(worldPoints - [x, y], 2, 2);
    """
    if failed
        @error "extrinsic image is of too low quality, select another time-stamp with a clearer extrinsic image"
    else
        M = LinearMap(SMatrix{3,3, Float64}(tform'))
        tform = PerspectiveMap() ∘ M ∘ push1
        itform = PerspectiveMap() ∘ inv(M) ∘ push1
        ϵ = quantile(errors, [0, 0.5, 1])
        (; tform, itform, ϵ)
    end
end

"""
    buildcalibration(check, extrinsic, ::Missing)

A convinience function for building a calibration object from just a single checkerboard 
image (i.e. extrinsic). 
"""
buildcalibration(check, extrinsic, ::Missing) = buildcalibration(check, extrinsic)

push1(x) = push(x, 1.0)

"""
    buildcalibration(check, extrinsic, intrinsic)

Build a calibration object from extrinsic and intrinsic images. `check` 
is the width of the checker, `extrinsic` is the file name of the image, `intrinsic` is a
collection of the intrinsic image files.
"""
function buildcalibration(check, extrinsic::AbstractString, intrinsic::AbstractVector)
    mat"""
    warning('off','all')
    [imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints($intrinsic);
    extrinsicI = imread($extrinsic);
    $sz = size(extrinsicI);
    worldPoints = generateCheckerboardPoints(boardSize, $check);
    %%
    params = estimateCameraParameters(imagePoints, worldPoints, 'ImageSize', $sz, 'EstimateSkew', true, 'NumRadialDistortionCoefficients', 3, 'EstimateTangentialDistortion', true, 'WorldUnits', 'cm');
    n = size(imagePoints, 3);
    errors_ = zeros(n,1);
    for i = 1:n
        [R,t] = extrinsics(imagePoints(:,:,i), worldPoints, params);
        newWorldPoints = pointsToWorld(params, R, t, imagePoints(:,:,i));
        errors_(i) = mean(vecnorm(worldPoints - newWorldPoints, 1, 2));
    end
    kill = errors_ > 1;
    while any(kill)
        imagePoints(:,:,kill) = [];    
        params = estimateCameraParameters(imagePoints, worldPoints, 'ImageSize', $sz, 'EstimateSkew', true, 'NumRadialDistortionCoefficients', 3, 'EstimateTangentialDistortion', true, 'WorldUnits', 'cm');
        n = size(imagePoints, 3);
        errors_ = zeros(n,1);
        for i = 1:n
            [R,t] = extrinsics(imagePoints(:,:,i), worldPoints, params);
            newWorldPoints = pointsToWorld(params, R, t, imagePoints(:,:,i));
            errors_(i) = mean(vecnorm(worldPoints - newWorldPoints, 1, 2));
        end
        kill = errors_ > 1;
    end
    $errors = errors_;
    %%
    imUndistorted = undistortImage(extrinsicI, params);
    MinCornerMetric = 0.15;
    xy = detectCheckerboardPoints(imUndistorted, 'MinCornerMetric', MinCornerMetric);
    MinCornerMetric = 0.;
    i = 0;
    while size(xy,1) ~= size(worldPoints, 1) && i < 25
        MinCornerMetric = MinCornerMetric + 0.05;
        i = i + 1;
        xy = detectCheckerboardPoints(imUndistorted, 'MinCornerMetric', MinCornerMetric);
    end
    $failed = size(xy,1) ~= size(worldPoints, 1)
    """
    if failed
        @error "extrinsic image is of too low quality, select another time-stamp with a clearer extrinsic image"
    else
        h, w = Int.(sz[1:2])
        _image = [(x, y) for x in 1:w for y in 1:h]
        image = hcat(first.(_image), last.(_image))
        mat"""
        [R,t] = extrinsics(xy,worldPoints,params);
        $world = pointsToWorld(params, R, t, $image);
        """
        _tform = interpolate(reshape(V2.(eachrow(world)), h, w)', BSpline(Linear()))
        tform = splat(extrapolate(scale(_tform, 1:w, 1:h), Flat()))
        mx, Mx = extrema(world[:,1])
        my, My = extrema(world[:,2])
        xs = mx:Mx
        ys = my:My
        worldPoints = vcat(([x y 0.0] for x in xs for y in ys)...)
        mat"""
        $projectedPoints = worldToImage(params,R,t,$worldPoints);
        """
        _itform = interpolate(reshape(V2.(eachrow(projectedPoints)), length(ys), length(xs))', BSpline(Linear()))
        itform = splat(extrapolate(scale(_itform, xs, ys), Flat()))
        ϵ = quantile(errors, [0, 0.5, 1])
        (; tform, itform, ϵ)
    end
end
