module CameraCalibrations

using Statistics, LinearAlgebra
using PythonCall, FileIO, StaticArrays, CoordinateTransformations, Rotations, Polynomials, Colors, ImageCore
using ImageTransformations, ImageDraw

using Optim

export Calibration, Camera, SV
export image2real, real2image, gof, remove_errored!, plot
export zero_k₁, zero_k₂, zero_k₃, zero_tangential

#TODO: 
# - generate known set of images for tests
# - Can I really not thread/parallel the corner detection?
# - code coverage
# - documentation
# - invalid images where the corners are outside the distortion model
# - discourse post

# using CondaPkg
# CondaPkg.add.(["numpy", "opencv"])

"""
zero_tangential # Tangential distortion coefficients (p1,p2) are set to zeros and stay zero.
zero_k₁, zero_k₂, zero_k₃ # The corresponding radial distortion coefficient is set to zero.
"""
@enum LensDistortion zero_k₁ zero_k₂ zero_k₃ zero_tangential

const cv2 = PythonCall.pynew()
const np = PythonCall.pynew()
const lens_distortions = Dict{LensDistortion, Any}()

function __init__()
    PythonCall.pycopy!(cv2, pyimport("cv2"))
    PythonCall.pycopy!(np, pyimport("numpy"))
    lens_distortions[zero_k₁] = cv2.CALIB_FIX_K1
    lens_distortions[zero_k₂] = cv2.CALIB_FIX_K2
    lens_distortions[zero_k₃] = cv2.CALIB_FIX_K3
    lens_distortions[zero_tangential] = cv2.CALIB_ZERO_TANGENT_DIST
end


const SV = SVector{2, Float64}

mutable struct Calibration
    files::Vector
    n_corners::Tuple{Int, Int}
    checker_size::Float64
    images_points::Vector{Matrix{SV}}
    object_points::Matrix{SV}
    image_size::Tuple{Int, Int}
    distortion::Vector{LensDistortion}
end

struct Camera
    pixel_size::Float64
    distortion::NamedTuple
    rotation_vecs::Vector{SVector{3, Float64}}
    translation_vecs::Vector{SVector{3, Float64}}
    focal_length::Tuple{Float64, Float64}
    principal_point::SV
end

push0(x) = push(x, 0)

"""
    XYZ(x, y, z)
An alias for a static vector of three, x, y, and z, indicating a real-world coordinate. Note that `x` is equivalent to the `column` in `RowCol` and the `y` is equivalent to the `row`.
"""

include("detect_fit.jl")
include("buildcalibrations.jl")
include("plot_calibration.jl")

end
