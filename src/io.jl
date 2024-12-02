function Serde.SerJson.ser_type(::Type{AM}, v::SDiagonal{2, Float64})
    return v.diag
end

function Serde.SerJson.ser_type(::Type{LM3}, v::SDiagonal{3, Float64})
    return v.diag
end

function Serde.SerJson.ser_type(::Type{AMext}, v::RotationVec{Float64})
    return [v.sx, v.sy, v.sz]
end

function Serde.deser(::Type{AM}, ::Type{SVector{2, Float64}}, v::Vector{Any})
    return SVector{2, Float64}(v)
end

function Serde.deser(::Type{AMext}, ::Type{SVector{2, Float64}}, v::Vector{Any})
    return SVector{2, Float64}(v)
end

function Serde.deser(::Type{AM}, ::Type{SDiagonal{2, Float64}}, v::Vector{Any})
    return Diagonal(SVector{2, Float64}(v))
end

function Serde.deser(::Type{LM3}, ::Type{SDiagonal{3, Float64}}, v)
    return Diagonal(SVector{3, Float64}(v))
end

function Serde.deser(::Type{AMext}, ::Type{RotationVec{Float64}}, v::Vector{Any})
    return RotationVec(v...)
end

# function Serde.deser(::Type{@NamedTuple{n::Int64, reprojection::Float64, projection::Float64, distance::Float64, inverse::Float64}}, dict::Dict{String, Any})
#     ks = (:n, :reprojection, :projection, :distance, :inverse)
#     return (; (k => dict[String(k)] for k in ks)...)
# end

struct CalibrationIO
    intrinsic::AM
    extrinsics::Vector{AMext}
    scale::LM3
    k::Float64
    files::Vector{String}
end

function load(file)
    cio = deser_json(CalibrationIO, read(file, String))
    distort(rc) = lens_distortion(rc, cio.k)
    real2image = .∘(Ref(cio.intrinsic), distort, Ref(PerspectiveMap()), cio.extrinsics, Ref(cio.scale))
    inv_scale, inv_extrinsics, inv_perspective_maps, inv_distort, inv_intrinsic = CameraCalibrationMeta.img2obj(cio.intrinsic, cio.extrinsics, cio.scale, cio.k)
    image2real = .∘(Ref(inv_scale), inv_extrinsics, inv_perspective_maps, inv_distort, Ref(inv_intrinsic))
    return Calibration(cio.intrinsic, cio.extrinsics, cio.scale, cio.k, cio.files, real2image, image2real)
end

save(file, cio::CalibrationIO) = open("$file.calib", "w") do io
    print(io, to_json(cio))
end
save(file, c::Calibration) = save(file, CalibrationIO(c.intrinsic, c.extrinsics, c.scale, c.k, c.files))


