using CameraCalibrations
using Test

n_corners = (5, 8)
dir = "example"
files = readdir(dir; join=true)
checker_size = 1
calib = Calibration(files, n_corners, checker_size)
cam = Camera(calib)

@testset "Accuracy" begin
    ϵ = gof(calib, cam)
    @test all(<(0.1), ϵ)
end

