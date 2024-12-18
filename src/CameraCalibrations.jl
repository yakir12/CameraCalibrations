module CameraCalibrations

using LinearAlgebra, Statistics, Random
import ImageIO, FileIO
using OpenCV, ImageBase, StaticArrays, ImageDraw, ImageTransformations
using Rotations, CoordinateTransformations, Polynomials

using JSON3, StructTypes

include("meta.jl")
include("io.jl")
include("detect_fit.jl")
include("buildcalibrations.jl")
include("plot_calibration.jl")

const CRITERIA = OpenCV.TermCriteria(OpenCV.TERM_CRITERIA_EPS + OpenCV.TERM_CRITERIA_MAX_ITER, 30, 0.001)

export fit, Calibration, RowCol, XYZ, rectification

end
