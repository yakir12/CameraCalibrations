module CameraCalibrations

using Colors, CoordinateTransformations, FileIO, ImageCore, ImageDraw, ImageTransformations, PythonCall, Rotations, StaticArrays
using LinearAlgebra, Statistics

export Calibration, Camera, SV
export image2real, real2image, gof, remove_errored!, plot
export zero_k₁, zero_k₂, zero_k₃, zero_tangential

#TODO: 
# - generate known set of images for tests
# - Can I really not thread/parallel the corner detection?
# - code coverage
# - invalid images where the corners are outside the distortion model
# - discourse post

# using CondaPkg
# CondaPkg.add.(["numpy", "opencv"])

"""
Lens distortion is controlled with these enums
zero_tangential # Tangential distortion coefficients (p₁, p₂) are set to zeros and stay zero.
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

include("detect_fit.jl")
include("buildcalibrations.jl")
include("plot_calibration.jl")

end
