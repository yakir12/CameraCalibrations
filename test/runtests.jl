using CameraCalibrations
using Test
using Aqua
using FileIO, StaticArrays, PaddedViews, ColorTypes

function index2bw(ij::CartesianIndex) 
    i, j = Tuple(ij)
    isodd(i) ? isodd(j) : !isodd(j)
end

function generate_checkerboard(n_corners, n) 
    xys = [n .* SVector{2, Float32}(Tuple(ij)) - SVector(0.5, 0.5) for ij in CartesianIndices(StepRange.(n_corners .+ 1, -1, 2))]
    img = index2bw.(CartesianIndices(n_corners .+ 1))
    imgl = kron(PaddedView(true, img, UnitRange.(0, n_corners .+ 2)), ones(Int, n, n))
    return xys, imgl
end

@testset "CameraCalibrations.jl" begin
    @testset "Code quality (Aqua.jl)" begin
        Aqua.test_all(CameraCalibrations; piracies = false)
    end

    @testset "Real data" begin
        n_corners = (5, 8)
        dir = joinpath(@__DIR__(), "example")
        files = readdir(dir; join=true)
        checker_size = 1
        c, (n, ϵ...) = fit(files, n_corners, checker_size)

        @testset "Accuracy" begin
            # @show n, ϵ
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

    @testset "detect corners" begin
        for _ in 1:100
            n_corners = (rand(4:15), rand(4:15))
            if isequal(isodd.(n_corners)...)
                n_corners = (n_corners[1] + 1, n_corners[2])
            end
            ratio = rand(20:100)
            xys, img = generate_checkerboard(n_corners, ratio)
            sz = size(img)
            xys2 = mktempdir() do path
                file = joinpath(path, "img.png")
                FileIO.save(file, Gray.(img))
                _, xys2 = CameraCalibrations._detect_corners(file, n_corners, sz)
                return xys2
            end
            @test xys == xys2
        end
    end
end
