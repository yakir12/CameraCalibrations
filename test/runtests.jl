using CameraCalibrations
using Test
using Aqua
using FileIO, StaticArrays, PaddedViews, ColorTypes, LinearAlgebra, JSON3

function index2bw(ij::CartesianIndex) 
    i, j = Tuple(ij)
    isodd(i) ? isodd(j) : !isodd(j)
end

function generate_checkerboard(n_corners, n) 
    xys = [n .* SVector{2, Float32}(Tuple(ij)) - SVector(0.5, 0.5) for ij in CartesianIndices(StepRange.(n_corners .+ 1, -1, 2))]
    img = index2bw.(CartesianIndices(n_corners .+ 1))
    imgl = kron(PaddedView(true, img, UnitRange.(0, n_corners .+ 2)), ones(Int, n, n))
    # imgw = imfilter(imgl, Kernel.gaussian(2))
    return xys, imgl
end

function compare(n_corners, ratio)
    xys, img = generate_checkerboard(n_corners, ratio)
    sz = size(img)
    mktempdir() do path
        file = joinpath(path, "img.png")
        FileIO.save(file, Gray.(img))
        res = CameraCalibrations._detect_corners(file, n_corners, sz)
        return !ismissing(res) && all(<(1), norm.(xys .- res[2]))
    end
end

function attempt_compare(n_corners, ratio)
    for attempt in 1:3
        if compare(n_corners, ratio)
            return true
        else
            @warn "attempt $attempt failed"
        end
    end
    return false
end

function attempt_fit(files, n_corners, checker_size)
    for attempt in 1:3
        c, (n, ϵ...) = fit(files, n_corners, checker_size)
        if all(<(1), ϵ)
            return c, ϵ
        else
            @warn "attempt $attempt failed"
        end
        if attempt == 3
            return c, ϵ
        end
    end
end


@testset "CameraCalibrations.jl" begin
    @testset "Code quality (Aqua.jl)" begin
        Aqua.test_all(CameraCalibrations; piracies = false)
    end

    @testset "Detect corners" begin
        @testset "In artificial images" begin
            for w in 13:15, h in 13:15, ratio in 95:100
                # for w in 5:15, h in 5:15, ratio in 50:100
                if isodd(w) ≠ isodd(h)
                    n_corners = (w, h)
                    @test attempt_compare(n_corners, ratio)
                end
            end
        end

        @testset "In real images" begin
            dir = joinpath(@__DIR__(), "example")
            files = filter(file -> last(splitext(file)) == ".png", readdir(dir, join = true))
            sz = size(FileIO.load(first(files)))
            n = length(files)
            n_corners = (5, 8)
            target = JSON3.read(joinpath(dir, "corners.json"))
            for file in files
                res = CameraCalibrations._detect_corners(file, n_corners, sz)
                target_corners = target[basename(file)]
                @test !ismissing(res) && all(<(1), norm.(target_corners .- vec(res[2])))
            end
        end
    end

    @testset "Full calibration" begin
        n_corners = (5, 8)
        dir = joinpath(@__DIR__(), "example")
        files = filter(file -> last(splitext(file)) == ".png", readdir(dir, join = true))
        checker_size = 1
        c, ϵ = attempt_fit(files, n_corners, checker_size)

        @testset "Accuracy" begin
            @test all(<(1), ϵ)
        end

        @testset "Rectification" begin
            extrinsic_index = 1
            extrinsic_file = files[extrinsic_index]
            f = rectification(c, 1)
            i = RowCol(1,2)
            @test f(i) == c(i, 1)[[1,2]]
        end
    end

    @testset "IO" begin
        for _ in 1:100
            org = rand(CameraCalibrations.CalibrationIO)
            copy = mktempdir() do path
                file = joinpath(path, "calibration.json")
                CameraCalibrations.save(file, org)
                CameraCalibrations.load(file)
            end
            @test org.files == copy.files
        end
    end

end
