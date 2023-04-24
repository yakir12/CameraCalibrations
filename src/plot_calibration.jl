function get_axes(s, c, extrinsic_index, ratio, n_corners, sz)

# TODO: clean all the ratio, sizing, shit. streamline this a bit more
#
    # extrinsic_index = 1
    # ratio = CameraCalibrations.get_ratio(cd1.images_points[extrinsic_index], c.checker_size)
    # sz = cd1.image_size
    # f = ∘(LinearAlgebra.norm_sqr, CameraCalibrations.PerspectiveMap(), CameraCalibrations.extrinsic(c, extrinsic_index), CameraCalibrations.scale(c), CameraCalibrations.push0)

    w = round.(Int, ratio .* c.checker_size .* (n_corners .- 1))
    min_x, min_y = round.(Int, (w .- reverse(sz)) ./ 2)
    max_x, max_y = (min_x, min_y) .+ reverse(sz) .- 1
    xs = min_x:max_x
    ys = min_y:max_y
    axs = (xs, ys)

    c.k ≥ 0 && return axs

    f = ∘(>(-1/3/c.k), LinearAlgebra.norm_sqr, PerspectiveMap(), extrinsic(c, extrinsic_index), scale(c), push0, inv(s))

    x1 = 1
    y1 = 1
    x2 = w[1]
    y2 = w[2]
    for _ in 1:1000
        x1 -= 1
        y1 -= 1
        x2 += 1
        y2 += 1
        corners = XY.([(x1, y1), (x1, y2), (x2, y1), (x2, y2)])
        # @show corners
        # @show f.(corners)
        if any(f, corners)
            # asdjkhrasdkjh
        # try
        #     real2image(c, extrinsic_index).(corners)
        # catch ex
            # @show ex
            return (x1 + 1:x2 - 1, y1 + 1:y2 - 1)
        end
    end


    return (x1 :x2 , y1 :y2 )



    # ix_min = findfirst(x -> all(y -> f(XY(x, y)) < -1/3/c.k, ys), xs)
    #
    # ix_max = findlast(x -> all(y -> f(XY(x, y)) < -1/3/c.k, ys), xs)
    # iy_min = findfirst(y -> all(x -> f(XY(x, y)) < -1/3/c.k, xs), ys)
    # iy_max = findlast(y -> all(x -> f(XY(x, y)) < -1/3/c.k, xs), ys)
    #
    # return (xs[ix_min]:xs[ix_max], ys[iy_min]:ys[iy_max])
end

function get_ratio(imgpoints, checker_size)
    l = mean(1:2) do dims
        mean(norm, diff(imgpoints; dims))
    end
    l/checker_size
end

function image_transformations(cd, c, extrinsic_index; ratio=get_ratio(cd.images_points[extrinsic_index], c.checker_size))
    s = LinearMap(SDiagonal{2}(ratio*I))
    tform =  (xy -> real2image(c, extrinsic_index)(xy)) ∘ inv(s)
    itform = s ∘ (rc -> image2real(c, extrinsic_index)(rc))
    axs = get_axes(s, c, extrinsic_index, ratio, cd.n_corners, cd.image_size)
    return (; tform, itform, axs)
end

get_img(file::String) = RGB.(load(file))
get_img(img) = img

function draw_crosses!(file, imgpoint, n1, color)
    ij = vec([round.(Int, rc) for rc in imgpoint])
    img = get_img(file)
    radius = round(Int, norm(ij[1] - ij[n1])/n1/5) # radius relative to checkerboard's aparent size
    draw!(img, Cross.(Point.(CartesianIndex.(Tuple.(ij))), radius), color)
end

"""
    plot
Save to locally created directory `debug` all the calibration's images after rectification. Blue crosses indicate detetcted corners, and red crosses indicate their rectificated real-world locations. Ideally, both crosses' centers are right on top of each other.
"""
function plot(cd::CalibrationData, c::Calibration)
    dir = mkpath("debug")
    for (i, (file, imgpoints)) in enumerate(zip(cd.files, cd.images_points))
        img = draw_crosses!(file, imgpoints, cd.n_corners[1], colorant"red")
        tform, itform, axs = image_transformations(cd, c, i)
        imgw = warp(img, tform, axs)
        draw_crosses!(imgw, itform.(imgpoints), cd.n_corners[1], colorant"blue")
        save(joinpath(dir, basename(file)), parent(imgw))
    end
end
