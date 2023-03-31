using CameraCalibrations
using Test

@testset "Accuracy" begin
    n_corners = (5, 8)
    dir = "example"
    files = readdir(dir; join=true)
    checker_size = 1
    extrinsic_file = files[6]

    c = Calibration(files, n_corners, checker_size, extrinsic_file)
    ϵ = calculate_errors(c)

    @test all(<(1), ϵ)
end

@testset "Assertions" begin
    n_corners = (5, 8)
    dir = "example"
    files = readdir(dir; join=true)
    checker_size = 1
    extrinsic_file = files[6]
    push!(files, extrinsic_file)

    @test_throws AssertionError Calibration(files, n_corners, checker_size, extrinsic_file)

    files = readdir(dir; join=true)
    extrinsic_file = files[6] * "does not exist in files"

    @test_throws AssertionError Calibration(files, n_corners, checker_size, extrinsic_file)
end
