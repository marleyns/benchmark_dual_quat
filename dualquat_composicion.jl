#=
===================================================================
Composicion de CUATERNIONES DUALES para la pose de la BRIDA del UR10
Analoga a la composicion de matrices homogeneas: T = T1*T2*T3*T4*T5*T6

Los datos (q_1..q_6, t_1..t_6) provienen DIRECTAMENTE de la salida del
script de Python "ur10_tcp_wrt_reference_robodk.py" (funcion
print_quaternion_translation), corrida con los angulos de joint
reales de la estacion (no la corrida "home"/identidad).

API: Quaternions.jl + ForwardDiff.jl
===================================================================
=#

using Quaternions
using ForwardDiff
using LinearAlgebra
using Random

# ===================================================================
# 1) Funciones: motodual(), translation(), dualmoto())
# ===================================================================

const DualQuaternion{T} = Quaternion{ForwardDiff.Dual{Nothing,T,1}}

purequat(p::AbstractVector) = quat(false, @views(p[begin:begin+2])...)
dual(x::Real, v::Real) = ForwardDiff.Dual(x, v)

function dualquat(_q0::Union{Real,Quaternion}, _qe::Union{Real,Quaternion})
    q0 = quat(_q0)
    qe = quat(_qe)
    Quaternion(
        dual(real(q0), real(qe)),
        dual.(imag_part(q0), imag_part(qe))...,
    )
end

function primal(d::DualQuaternion)
    return Quaternion(
        ForwardDiff.value(real(d)),
        ForwardDiff.value.(imag_part(d))...,
    )
end

function tangent(d::DualQuaternion)
    return Quaternion(
        ForwardDiff.partials(real(d), 1),
        ForwardDiff.partials.(imag_part(d), 1)...,
    )
end

function dualconj(d::DualQuaternion)
    de = tangent(d)
    return dualquat(conj(primal(d)), quat(-real(de), imag_part(de)...))
end

rotation_part(d::DualQuaternion) = primal(d)
translation_part(d::DualQuaternion) = dualquat(true, conj(rotation_part(d)) * tangent(d))

function rotmatrix_from_quat(q::Quaternion)
    sx, sy, sz = 2q.s * q.v1, 2q.s * q.v2, 2q.s * q.v3
    xx, xy, xz = 2q.v1^2, 2q.v1 * q.v2, 2q.v1 * q.v3
    yy, yz, zz = 2q.v2^2, 2q.v2 * q.v3, 2q.v3^2
    r = [1 - (yy + zz)     xy - sz     xz + sy;
              xy + sz  1 - (xx + zz)     yz - sx;
              xz - sy      yz + sx  1 - (xx + yy)]
    return r
end

# motodual() (construye un cuaternion dual a partir de una
# matriz homogenea 4x4). 
function motodual(T::Matrix{Float64})
    R = T[1:3, 1:3]
    t = T[1:3, 4]
    q_r = qrotation(R)
    t_quat = Quaternions.Quaternion(0.0, t[1], t[2], t[3])
    q_l = 0.5 * (t_quat * q_r)
    dq = dualquat(q_r, q_l)
    return dq
end

# Construcción de un cuaternion dual directamente desde un cuaternion de
# rotacion (q_r) y un vector de traslacion (t).
function motodual_from_quat(q_r::Quaternion, t::AbstractVector)
    t_quat = purequat(t)
    q_l = 0.5 * (t_quat * q_r)
    return dualquat(q_r, q_l)
end

# FUNCION TRASLACION 
function translation(d::DualQuaternion)
    q_r = primal(d)
    q_l = tangent(d)          # parte dual q_l
    # t = 2 * q_l ⊗ conj(q_r)
    t_quat = 2.0 * (q_l * conj(q_r))
    return [t_quat.v1, t_quat.v2, t_quat.v3]
end

# FUNCION TRANSFORMATIONMATRIX 
function dualmoto(d::DualQuaternion)
    R = rotmatrix_from_quat(rotation_part(d))
    t = translation(d)
    T = zeros(Float64, 4, 4)
    T[1:3, 1:3] .= R
    T[1:3, 4]   .= t
    T[4, 1:3]   .= 0.0
    T[4, 4]     = 1.0
    return T
end

randdualquat(rng::AbstractRNG, T=Float64) = dualquat(rand(rng, Quaternion{T}), rand(rng, Quaternion{T}))
randdualquat(T=Float64) = randdualquat(Random.GLOBAL_RNG, T)

# Normalizacion de un cuaternion dual, para eliminar el drift
# numerico que se acumula al multiplicar varios cuaterniones duales.
function normalize_dualquat(d::DualQuaternion)
    qr = primal(d)
    ql = tangent(d)
    n = abs(qr)                      # norma del cuaternion de rotacion
    qr_n = qr / n
    ql_n = ql / n
    corr = real(ql_n * conj(qr_n))   # componente espuria (deberia ser 0)
    ql_n = ql_n - corr * qr_n        # se proyecta y se elimina
    return dualquat(qr_n, ql_n)
end

