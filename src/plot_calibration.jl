get_img(file::String) = RGB.(load(file))
get_img(img) = img

function draw_crosses!(file, imgpoint, n1, color=colorant"red")
    ij = vec([round.(Int, rc) for rc in imgpoint] )
    img = get_img(file)
    radius = round(Int, norm(ij[1] - ij[n1])/n1/5) # radius relative to checkerboard's aparent size
    draw!(img, Cross.(Point.(CartesianIndex.(Tuple.(ij))), radius), color)
end

function plot(c::Calibration)
    dir = mkpath("debug")
    for (file, imgpoint) in zip(c.files, c.imgpointss)
        img = draw_crosses!(file, imgpoint, c.n_corners[1])
        save(joinpath(dir, basename(file)), img)
    end

    imgpoints = c.imgpointss[c.extrinsic_index]
    file = c.files[c.extrinsic_index]
    img = draw_crosses!(file, imgpoints, c.n_corners[1])

    ratio = get_ratio(imgpoints, c.checker_size)
    s = LinearMap(SDiagonal{2}(ratio*I))
    tform =  c ∘ Base.Fix2(push, 0) ∘ inv(s)
    axs = get_axes(ratio, c.checker_size, c.n_corners, c.sz)
    imgw = warp(img, tform, axs)

    itform = s ∘ pop ∘ c
    draw_crosses!(imgw, itform.(imgpoints), c.n_corners[1], colorant"blue")
    _, ext = splitext(file)
    save(joinpath(dir, "extrinsic$ext"), imgw)
end
