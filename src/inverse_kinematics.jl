using StaticArrays

include("jacobian.jl")

"""
    inverse_kinematics!(q, xp, qp, J)

Calculate the velocity level inverse kinematics for Kinova Gen3 robot.

This method reduces the number of allocations

TODO LinearAlgebra.pinv cause allocations during runtime

# Arguments
- `q::SVector{7, Float64}`: the joint angles of the robot [rad]
- `xp::SVector{6, Float64}`: the end-effector velocities of the robot [m/s]
- `qp::MVector{6, Float64}`: the joint space velocities of the robot [rad/s]
- `J::MMatrix{6, 7, Float64}`: the Jacobian matrix of the robot []

# Examples
```julia-repl
julia> inverse_kinematics(zeros(SVector{7}), zeros(SVector{6}, zeros(MVector{6})), zeros(MMatrix{6, 7}))
[0.0, 0.0 0.0 0.0, 0.0, 0.0, 0.0]
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
```julia-repl
julia> inverse_kinematics(SVector(0., 0., 0., 0., 0., 0., 0.), SVector(0., 0., 0., 0., 0., 0.))
[0.0, 0.0 0.0 0.0, 0.0, 0.0, 0.0]
```
"""
function inverse_kinematics(q::SVector{7, Float64}, xp::SVector{6, Float64})
    qp = zeros(MVector{7})
    J = zeros(MMatrix{6, 7})
    inverse_kinematics!(q, xp, qp, J)

    return qp
end