module KinovaGen3

# Write your package code here.

# Analytical modeling of the robot

# Kinematics functions
include("forward_kinematics.jl")
# include("inverse_kinematics.jl")
# include("jacobian.jl")

# Dynamics functions
# include("mass_matrix.jl")
# include("coriolis.jl")
# include("gravity.jl")

# Kinematics of the robot
export
    forward_kinematics!
    forward_kinematics
# export inverse_kinematics
# export jacobian

# Dynamics of the robot
# export mass_matrix
# export coriolis
# export gravity

end
