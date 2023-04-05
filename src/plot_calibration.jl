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

function image_transformations(c, extrinsic_index; ratio=get_ratio(c.imgpointss[extrinsic_index], c.checker_size))
    s = LinearMap(SDiagonal{2}(ratio*I))
    tform =  c.real2image[extrinsic_index] ∘ Base.Fix2(push, 0) ∘ inv(s)
    itform = s ∘ pop ∘ c.image2real[extrinsic_index]
    axs = get_axes(ratio, c.checker_size, c.n_corners, c.sz)
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

function plot(c::Calibration)
    dir = mkpath("debug")
    for (file, imgpoints) in zip(c.files, c.imgpointss)
        img = draw_crosses!(file, imgpoints, c.n_corners[1], colorant"red")
        tform, itform, axs = image_transformations(c, findfirst(==(file), c.files))
        imgw = warp(img, tform, axs)
        draw_crosses!(imgw, itform.(imgpoints), c.n_corners[1], colorant"blue")
        save(joinpath(dir, basename(file)), parent(imgw))
    end
end
