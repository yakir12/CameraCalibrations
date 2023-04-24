module CameraCalibrations

using Statistics, LinearAlgebra
using PythonCall, FileIO, StaticArrays, CoordinateTransformations, Rotations, Polynomials, Colors
using ImageTransformations, ImageDraw

export CalibrationData, Calibration, RowCol, XY, calculate_errors, rectification, plot, improve

# using CondaPkg
# CondaPkg.add.(["numpy", "opencv"])

const cv2 = PythonCall.pynew()
const np = PythonCall.pynew()

function __init__()
    PythonCall.pycopy!(cv2, pyimport("cv2"))
    PythonCall.pycopy!(np, pyimport("numpy"))
end

struct RowCol{T} <: FieldVector{2, T}
    row::T
    col::T
end
StaticArrays.similar_type(::Type{<:RowCol}, ::Type{T}, s::Size{(2,)}) where {T} = RowCol{T}

struct XY{T} <: FieldVector{2, T}
    x::T
    y::T
end
StaticArrays.similar_type(::Type{<:XY}, ::Type{T}, s::Size{(2,)}) where {T} = XY{T}

struct CalibrationData
    files::Vector
    n_corners::Tuple{Int, Int}
    checker_size::Float64
    images_points::Vector{Matrix{RowCol}}
    object_points::Matrix{XY}
    image_size::Tuple{Int, Int}
    with_distortion::Bool
end

struct Calibration
    checker_size::Float64
    k::Float64
    rotation_vecs::Vector{SVector{3, Float64}}
    translation_vecs::Vector{SVector{3, Float64}}
    frow::Float64
    fcol::Float64
    crow::Float64
    ccol::Float64
end

push0(x) = push(x, 0)

"""
    RowCol(row, col)
An alias for a static vector of two, row and column, indicating a cartesian coordinate in an image/matrix.
"""

"""
    XYZ(x, y, z)
An alias for a static vector of three, x, y, and z, indicating a real-world coordinate. Note that `x` is equivalent to the `column` in `RowCol` and the `y` is equivalent to the `row`.
"""

include("detect_fit.jl")
include("buildcalibrations.jl")
include("plot_calibration.jl")

end
