using StaticArrays

"""
    forward_kinematics!(q, x, R)

Calculate the position level forward kinematics for Kinova Gen3 robot.

This function does not perform dynamic memory allocation.

# Arguments
- `q::SVector{7, Float64}`: the joint angles of the robot [rad]
- `x::MVector{3, Float64}`: the end-effector position of the robot [m]
- `R::MMatrix{3, 3, Float64}`: the end-effector orientation of the robot []

# Examples
```julia-repl
julia> q = SVector(0.0, π / 6, π / 3, π / 2, 0.0, π / 4, 0.0)
julia> x = zeros(MVector{3})
julia> R = zeros(MMatrix{3, 3})
julia> forward_kinematics!(q, x, R)
([0.32874611223407973, -0.393073312842223, 0.44415411959033596], [-0.6597396084411711 0.75 -0.047367172745376274; 0.6123724356957945 0.5000000000000001 -0.6123724356957946; -0.4355957403991577 -0.43301270189221924 -0.7891491309924313])
```
"""
function forward_kinematics!(q::SVector{7, Float64}, x::MVector{3, Float64}, R::MMatrix{3, 3, Float64})

    @boundscheck size(q, 1) == 7 || throw(DimensionMismatch("Size of the joint position vector is wrong!"))

    q1 = q[1]
    q2 = q[2]
    q3 = q[3]
    q4 = q[4]
    q5 = q[5]
    q6 = q[6]
    q7 = q[7]

    x0 = sin(q1)
    x1 = cos(q3)
    x2 = x0 .* x1
    x3 = sin(q2)
    x4 = cos(q1)
    x5 = cos(q4)
    x6 = cos(q2)
    x7 = sin(q3)
    x8 = x4 .* x7
    x9 = x6 .* x8
    x10 = cos(q5)
    x11 = x2 + x9
    x12 = x10 .* x11
    x13 = sin(q4)
    x14 = x0 .* x7
    x15 = x1 .* x4 .* x6 - x14
    x16 = sin(q5)
    x17 = x3 .* x4
    x18 = -x13 .* x17 + x15 .* x5
    x19 = x16 .* x18
    x20 = cos(q6)
    x21 = -x13 .* x15 - x17 .* x5
    x22 = x20 .* x21
    x23 = sin(q6)
    x24 = x10 .* x18 - x11 .* x16
    x25 = x0 .* x3
    x26 = x1 .* x4 - x14 .* x6
    x27 = x10 .* x26
    x28 = -x2 .* x6 - x8
    x29 = x13 .* x25 + x28 .* x5
    x30 = x16 .* x29
    x31 = x0 .* x3 .* x5 - x13 .* x28
    x32 = x20 .* x31
    x33 = x10 .* x29 - x16 .* x26
    x34 = x3 .* x7
    x35 = x5 .* x6
    x36 = x10 .* x34
    x37 = x1 .* x3
    x38 = x13 .* x37
    x39 = -x35 + x38
    x40 = x20 .* x39
    x41 = -x13 .* x6 - x37 .* x5
    x42 = x16 .* x41
    x43 = x10 .* x41 + x16 .* x34
    x44 = x23 .* x43
    x45 = sin(q7)
    x46 = x12 + x19
    x47 = cos(q7)
    x48 = x20 .* x24 + x21 .* x23
    x49 = x27 + x30
    x50 = x20 .* x33 + x23 .* x31
    x51 = -x36 + x42
    x52 = x20 .* x43 + x23 .* x39

    x[1] = -0.01175 * x0 - 0.0003501 * x12 + 0.31436 * x13 .* x15 - 0.0003501 * x19 - 0.01275 * x2 - 0.16743 * x22 + 0.16743 * x23 .* x24 + 0.31436 * x3 .* x4 .* x5 + 0.42076 * x3 .* x4 - 0.01275 * x9
    x[2] = 0.01275 * x0 .* x6 .* x7 - 0.01275 * x1 .* x4 + 0.31436 * x13 .* x28 + 0.16743 * x23 .* x33 - 0.31436 * x25 .* x5 - 0.42076 * x25 - 0.0003501 * x27 - 0.0003501 * x30 - 0.16743 * x32 - 0.01175 * x4
    x[3] = 0.01275 * x34 + 0.31436 * x35 + 0.0003501 * x36 - 0.31436 * x38 - 0.16743 * x40 - 0.0003501 * x42 + 0.16743 * x44 + 0.42076 * x6 + 0.28481

    R[1, 1] = -x45 .* x46 + x47 .* x48
    R[1, 2] = x45 .* x48 + x46 .* x47
    R[1, 3] = -x22 + x23 .* x24;
    R[2, 1] = -x45 .* x49 + x47 .* x50
    R[2, 2] = x45 .* x50 + x47 .* x49
    R[2, 3] = x23 .* x33 - x32;
    R[3, 1] = -x45 .* x51 + x47 .* x52
    R[3, 2] = x45 .* x52 + x47 .* x51
    R[3, 3] = -x40 + x44

    return (x, R)
end

"""
    forward_kinematics(q)

Calculate the position level forward kinematics for Kinova Gen3 robot.

# Arguments
- `q::SVector{7, Float64}`: the joint angles of the robot [rad]

# Examples
```jldoctest
julia> q = SVector(0.0, π / 6, π / 3, π / 2, 0.0, π / 4, 0.0)
julia> forward_kinematics(q)
([0.32874611223407973, -0.393073312842223, 0.44415411959033596], [-0.6597396084411711 0.75 -0.047367172745376274; 0.6123724356957945 0.5000000000000001 -0.6123724356957946; -0.4355957403991577 -0.43301270189221924 -0.7891491309924313])
```
"""
function forward_kinematics(q::SVector{7, Float64})
    @boundscheck size(q, 1) == 7 || throw(DimensionMismatch("Size of the joint position vector is wrong!"))

    x = zeros(MVector{3})
    R = zeros(MMatrix{3, 3})

    forward_kinematics!(q, x, R)

    return (x, R)
end
