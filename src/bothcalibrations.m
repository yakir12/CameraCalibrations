function [sz, cameraParams, R, t, errors] = bothcalibrations(check, extrinsic, intrinsic)
    intrinsicFileNames = intrinsic;
    [imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(intrinsicFileNames);
    intrinsicFileNames = intrinsicFileNames(imagesUsed);

    extrinsicImage = imread(extrinsic);
    [mrows, ncols, ~] = size(extrinsicImage);

    worldPoints = generateCheckerboardPoints(boardSize, check);

    [cameraParams, imagesUsed] = estimateCameraParameters(imagePoints, worldPoints, 'EstimateSkew', false, 'EstimateTangentialDistortion', false, 'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'centimeters', 'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], 'ImageSize', [mrows, ncols]);
    intrinsicFileNames = intrinsicFileNames(imagesUsed);

    n = length(intrinsicFileNames);
    errors2 = zeros(n, 1);
    for i = 1:n
        imOrig = imread(intrinsicFileNames{i});
        ips = undistortPoints(imagePoints(:,:, i), cameraParams);
        [R, t] = extrinsics(ips, worldPoints, cameraParams);
        ips2 = worldToImage(cameraParams, R, t, [worldPoints, zeros(size(worldPoints,1), 1)]);
        errors2(i) = mean(vecnorm(ips - ips2, 2, 2));
    end

    [~, i] = sort(errors2);
    if n < 5
        ni = 0;
    else
        ni = round(n*0.1);
    end
    keep = i(1:n - ni);
    errors = errors2(keep);
    intrinsicFileNames = intrinsicFileNames(keep);

    imagePoints = detectCheckerboardPoints(intrinsicFileNames);

    cameraParams = estimateCameraParameters(imagePoints, worldPoints, 'EstimateSkew', false, 'EstimateTangentialDistortion', false, 'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'centimeters', 'InitialIntrinsicMatrix', cameraParams.IntrinsicMatrix, 'InitialRadialDistortion', cameraParams.RadialDistortion, 'ImageSize', [mrows, ncols]);

    [im, newOrigin] = undistortImage(extrinsicImage, cameraParams);
    xy = detectCheckerboardPoints(im);

    i = 0;
    MinCornerMetric = 0.05;
    while size(xy,1) ~= size(worldPoints, 1) && i < 25
        MinCornerMetric = MinCornerMetric + 0.05;
        i = i + 1;
        xy = detectCheckerboardPoints(im, 'MinCornerMetric', MinCornerMetric);
    end
    disp('kaka')
    [R,t] = extrinsics(xy,worldPoints,cameraParams);

    sz = [mrows, ncols];
end
