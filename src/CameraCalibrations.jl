module CameraCalibrations

using Statistics, LinearAlgebra
using PythonCall, FileIO, StaticArrays, CoordinateTransformations, Rotations, Polynomials
using ImageTransformations, Colors, ImageDraw

export Calibration, RowCol, XYZ, calculate_errors, rectification, plot, improve

# using CondaPkg
# CondaPkg.add.(["numpy", "opencv"])

const cv2 = PythonCall.pynew()
const np = PythonCall.pynew()

function __init__()
    PythonCall.pycopy!(cv2, pyimport("cv2"))
    PythonCall.pycopy!(np, pyimport("numpy"))
end

"""
    RowCol(row, col)
An alias for a static vector of two, row and column, indicating a cartesian coordinate in an image/matrix.
"""
const RowCol = SVector{2, <: Real}

"""
    XYZ(x, y, z)
An alias for a static vector of three, x, y, and z, indicating a real-world coordinate. Note that `x` is equivalent to the `column` in `RowCol` and the `y` is equivalent to the `row`.
"""
const XYZ = SVector{3, <: Real}

include("detect_fit.jl")
include("buildcalibrations.jl")
include("plot_calibration.jl")

end
