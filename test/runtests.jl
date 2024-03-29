using CameraCalibrations
using Test

n_corners = (5, 8)
dir = "example"
files = readdir(dir; join=true)
checker_size = 1
c = Calibration(files, n_corners, checker_size)

@testset "Accuracy" begin
    n, ϵ... = calculate_errors(c)
    @test all(<(1), ϵ)
end

@testset "Rectification" begin
    extrinsic_index = 1
    extrinsic_file = files[extrinsic_index]
    f = rectification(c, 1)
    i = RowCol(1,2)
    @test f(i) == c(i, 1)[[1,2]]
end

