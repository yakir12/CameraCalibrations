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
    try
        intrinsicFileNames = $intrinsic
        % Detect checkerboards in images
        [imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(intrinsicFileNames);
        intrinsicFileNames = intrinsicFileNames(imagesUsed);

        % Read the first image to obtain image size
        extrinsicImage = imread($extrinsic);
        [mrows, ncols, ~] = size(extrinsicImage);
        $sz = [mrows, ncols];

        % Generate world coordinates of the corners of the squares
        worldPoints = generateCheckerboardPoints(boardSize, $check);

        % Calibrate the camera
        [cameraParams, imagesUsed] = estimateCameraParameters(imagePoints, worldPoints, ...
        'EstimateSkew', true, 'EstimateTangentialDistortion', true, ...
        'NumRadialDistortionCoefficients', 3, 'WorldUnits', 'centimeters', ...
        'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
        'ImageSize', [mrows, ncols]);
        intrinsicFileNames = intrinsicFileNames(imagesUsed);

        % Find reprojection error
        n = length(intrinsicFileNames);
        errors2 = zeros(n, 1);
        for i = 1:n
            imOrig = imread(intrinsicFileNames{i});
            [im, newOrigin] = undistortImage(imOrig, cameraParams);
            [ips,boardSize] = detectCheckerboardPoints(im);
            ips = [ips(:,1) + newOrigin(1), ips(:,2) + newOrigin(2)];
            [R, t] = extrinsics(ips, worldPoints, cameraParams);
            ips2 = worldToImage(cameraParams, R, t, [worldPoints, zeros(size(worldPoints,1), 1)]);
            errors2(i) = mean(vecnorm(ips - ips2, 2, 2));
        end

        % Delete the 10% worst cases
        [~, i] = sort(errors2);
        if n < 5
            ni = 0;
        else
            ni = round(n*0.1);
        end
        intrinsicFileNames = intrinsicFileNames(i(1:n - ni));
        $errors = errors2(i(1:n - ni));

        % Detect checkerboards in images
        imagePoints = detectCheckerboardPoints(intrinsicFileNames);

        % Calibrate the camera again
        cameraParams = estimateCameraParameters(imagePoints, worldPoints, ...
        'EstimateSkew', true, 'EstimateTangentialDistortion', true, ...
        'NumRadialDistortionCoefficients', 3, 'WorldUnits', 'centimeters', ...
        'InitialIntrinsicMatrix', cameraParams.IntrinsicMatrix, 'InitialRadialDistortion', cameraParams.RadialDistortion, ...
        'ImageSize', [mrows, ncols]);

        [im, newOrigin] = undistortImage(extrinsicImage, cameraParams);
        xy = detectCheckerboardPoints(im);

        % In case the single extrinsic image is not optimal
        i = 0;
        while size(xy,1) ~= size(worldPoints, 1) && i < 25
            MinCornerMetric = MinCornerMetric + 0.05;
            i = i + 1;
            xy = detectCheckerboardPoints(im, 'MinCornerMetric', MinCornerMetric);
        end


        [R,t] = extrinsics(xy,worldPoints,cameraParams);
        $success = true;
    catch ex
        $success = false;
    end
    """
    @assert success "extrinsic image is of too low quality, select another time-stamp with a clearer extrinsic image"

    h, w = Int.(sz[1:2])
    rows = 1:h
    cols = 1:w
    points = vcat(map.(hcat, rows, cols')...)

    mat"""
    $world = pointsToWorld(cameraParams, R, t, $points);
    """
    _tform = interpolate(reshape(V2.(eachrow(world)), h, w)', BSpline(Linear()))
    tform = splat(extrapolate(scale(_tform, 1:w, 1:h), Flat()))
    mx, Mx = extrema(world[:,1])
    my, My = extrema(world[:,2])
    xs = mx:Mx
    ys = my:My
    worldPoints = vcat(map.(hcat, xs, ys', 0.0)...)
    # worldPoints = vcat(([x y 0.0] for x in xs for y in ys)...)
    mat"""
    $projectedPoints = worldToImage(cameraParams,R,t,$worldPoints);
    """
    _itform = interpolate(reshape(V2.(eachrow(projectedPoints)), length(ys), length(xs))', BSpline(Linear()))
    itform = splat(extrapolate(scale(_itform, xs, ys), Flat()))
    ϵ = errors
    (; tform, itform, ϵ)
end
