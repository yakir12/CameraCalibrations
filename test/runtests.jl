using CameraCalibrations
using Test
using Aqua

@testset "CameraCalibrations.jl" begin
    @testset "Code quality (Aqua.jl)" begin
        Aqua.test_all(CameraCalibrations; piracies = false)
    end

    n_corners = (5, 8)
    dir = joinpath(@__DIR__(), "example")
    files = readdir(dir; join=true)
    checker_size = 1
    c, (_, ϵ...) = fit(files, n_corners, checker_size)

    @testset "Accuracy" begin
        @test all(<(2), ϵ)
    end

    @testset "Rectification" begin
        extrinsic_index = 1
        extrinsic_file = files[extrinsic_index]
        f = rectification(c, 1)
        i = RowCol(1,2)
        @test f(i) == c(i, 1)[[1,2]]
    end

    @testset "IO" begin
        for _ in 1:10
            org = rand(CameraCalibrations.CalibrationIO)
            copy = mktemp() do file, io
                CameraCalibrations.save(file, org)
                close(io)
                CameraCalibrations.load(file)
            end
            @test org.files == copy.files
        end
    end
end
