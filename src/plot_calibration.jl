function plotone!(img, tform, itform, image_points, radius, axs)
    crosses = [CirclePointRadius(Point(CartesianIndex(Tuple(round.(Int, point)))), radius) for point in image_points]
    draw!(img, vec(crosses), colorant"red")
    imgw = warp(img, tform, axs)
    crosses = [Cross(Point(CartesianIndex(Tuple(round.(Int, itform(point))))), radius) for point in image_points]
    draw!(imgw, vec(crosses), colorant"green")
    return imgw
end

"""
    plot
Save to locally created directory `debug` all the calibration's images after rectification. Blue crosses indicate detetcted corners, and red crosses indicate their rectificated real-world locations. Ideally, both crosses' centers are right on top of each other.
"""
function plot(calib::Calibration, cam::Camera)
    f = calib.image_size[2] / calib.n_corners[1] / 3
    radius = round(Int, f/4)
    ratio = f/calib.checker_size
    s = LinearMap(SDiagonal{2}(ratio*I))
    ws = round.(Int, f .* (reverse(calib.n_corners) .+ 1))
    axs = Tuple((-w:sz-w-1 for (w,sz) in zip(ws, calib.image_size)))
    dir = mkpath("debug")
    Threads.@threads for i in 1:length(calib.files)
        file = calib.files[i]
        image_points = calib.images_points[i]
        tform = real2image(cam, i) ∘ inv(s)
        itform = s ∘ image2real(cam, i)
        img = RGB.(load(file))
        imgw = plotone!(img, tform, itform, image_points, radius, axs)
        save(joinpath(dir, basename(file)), imgw)
    end
end
