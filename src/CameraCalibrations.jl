module CameraCalibrations

using Statistics, StaticArrays, CoordinateTransformations, ImageCore, ImageTransformations, Interpolations, MATLAB

export buildcalibration, calibrate

const V2 = SVector{2, Float64}

include("buildcalibrations.jl")

include("calibrate.jl")

end
