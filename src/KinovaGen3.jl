module KinovaGen3

# Write your package code here.

# Analytical modeling of the robot

# Kinematics functions
include("forward_kinematics.jl")
# include("jacobian.jl")
include("inverse_kinematics.jl")

# Dynamics functions
include("mass_matrix.jl")
include("coriolis.jl")
include("gravity.jl")

# Kinematics of the robot
export
    forward_kinematics!
    forward_kinematics
    jacobian!
    jacobian
    inverse_kinematics!
    inverse_kinematics

# Dynamics of the robot
export mass_matrix!
       mass_matrix
       coriolis!
       coriolis
       gravity!
       gravity

end
