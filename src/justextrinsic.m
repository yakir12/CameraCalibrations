function [ttform, errors] = justextrinsic(check, extrinsic)
    [imagePoints, boardSize] = detectCheckerboardPoints(extrinsic);

    worldPoints = generateCheckerboardPoints(boardSize, check);

    tform = fitgeotrans(imagePoints, worldPoints, 'projective');
    ttform = tform.T;

    ips2 = transformPointsInverse(tform, worldPoints);
    errors = mean(vecnorm(imagePoints - ips2, 2, 2));
end
