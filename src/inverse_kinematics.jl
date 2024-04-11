using StaticArrays

include("jacobian.jl")

"""
    inverse_kinematics!(q, xp, qp, J)

Calculate the velocity level inverse kinematics for Kinova Gen3 robot.

This method reduces the number of allocations

Note that LinearAlgebra.pinv cause allocations during runtime

# Arguments
- `q::SVector{7, Float64}`: the joint angles of the robot [rad]
- `xp::SVector{6, Float64}`: the end-effector velocities of the robot [m/s]
- `qp::MVector{6, Float64}`: the joint space velocities of the robot [rad/s]
- `J::MMatrix{6, 7, Float64}`: the Jacobian matrix of the robot []

# Examples
```jldoctest
julia> q = SVector(0.0, π / 6, π / 3, π / 2, 0.0, π / 4, 0.0)
julia> xp = SVector(5.519897654292758, 3.8454568904118693, 5.400775529650293, 0.05767137155933982, 0.09150292411182576, 0.33940753291008463)
julia> qp = zeros(MVector{7})
julia> J = zeros(MMatrix{6, 7})
julia> inverse_kinematics!(q, xp, qp, J)
7-element MVector{7, Float64} with indices SOneTo(7):
 -13.741946912596518
   0.24414409836241704
   5.390106099987782
 -17.3961161502307
   2.6801993763554823
  23.166518880789784
  -8.751150401752152
```
"""
function inverse_kinematics!(q::SVector{7, Float64}, xp::SVector{6, Float64}, qp::MVector{7, Float64}, J::MMatrix{6, 7, Float64})
    jacobian!(q, J)

    qp .= pinv(J) * xp

    return qp
end

"""
    inverse_kinematics(q, xp)

Calculate the velocity level inverse kinematics for Kinova Gen3 robot.

# Arguments
- `q::SVector{7, Float64}`: the joint angles of the robot [rad]
- `xp::SVector{6, Float64}`: the end-effector velocities of the robot [m/s]

# Examples
```jldoctest
julia> q = SVector(0.0, π / 6, π / 3, π / 2, 0.0, π / 4, 0.0)
julia> xp = SVector(5.519897654292758, 3.8454568904118693, 5.400775529650293, 0.05767137155933982, 0.09150292411182576, 0.33940753291008463)
julia> inverse_kinematics(q, xp)
7-element MVector{7, Float64} with indices SOneTo(7):
 -13.741946912596518
   0.24414409836241704
   5.390106099987782
 -17.3961161502307
   2.6801993763554823
  23.166518880789784
  -8.751150401752152
```
"""
function inverse_kinematics(q::SVector{7, Float64}, xp::SVector{6, Float64})
    qp = zeros(MVector{7})
    J = zeros(MMatrix{6, 7})
    inverse_kinematics!(q, xp, qp, J)

    return qp
end