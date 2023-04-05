module CameraCalibrations

using Statistics, LinearAlgebra
using PythonCall, FileIO, StaticArrays, CoordinateTransformations, Rotations, Polynomials
using ImageTransformations, Colors, ImageDraw

export Calibration, RowCol, XYZ, calculate_errors, rectification, plot

# using CondaPkg
# CondaPkg.add.(["numpy", "opencv"])

const cv2 = PythonCall.pynew()
const np = PythonCall.pynew()

function __init__()
    PythonCall.pycopy!(cv2, pyimport("cv2"))
    PythonCall.pycopy!(np, pyimport("numpy"))
end

const RowCol = SVector{2, <: Real}
const XYZ = SVector{3, <: Real}

include("buildcalibrations.jl")
include("plot_calibration.jl")

end
