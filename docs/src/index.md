# KinovaGen3.jl

```@meta
CurrentModule = KinovaGen3
```

## Summary

This package provides the analytical kinematic and dynamic modelling of the Kinova Gen3 industrial robot.

The mechanical properties of the robot such as the link lengths, masses, inertia are obtained from the [Kinova Gen3 manual](https://www.kinovarobotics.com/uploads/User-Guide-Gen3-R07.pdf).

In the kinematical model, the reference frames for each link of the robot is selected in accordance with the manual. See [page 193 of the manual](https://www.kinovarobotics.com/uploads/User-Guide-Gen3-R07.pdf#page=199) for the reference frames.

The aim of this package is to achieve an implementation level without making dynamic allocations when a function of the module is called. However, not all functions of the modules manages to prevent dynamic memory allocation.


## Kinematic Model

### Position-level Kinematics

```@docs
forward_kinematics!(q::SVector{7, Float64}, x::MVector{3, Float64}, R::MMatrix{3, 3, Float64})
forward_kinematics(q::SVector{7, Float64})
```

### Velocity-level Kinematics

```@docs
jacobian!(q::SVector{7, Float64}, J::MMatrix{6, 7, Float64})
jacobian(q::SVector{7, Float64})
inverse_kinematics!(q::SVector{7, Float64}, xp::SVector{6, Float64}, qp::MVector{7, Float64}, J::MMatrix{6, 7, Float64})
inverse_kinematics(q::SVector{7, Float64}, xp::SVector{6, Float64})
```

## Dynamics Model

```@docs
mass_matrix!(q::SVector{7, Float64}, M::MMatrix{7, 7, Float64})
mass_matrix(q::SVector{7, Float64})
coriolis!(q::SVector{7, Float64}, qp::SVector{7, Float64}, C::MVector{7, Float64})
coriolis(q::SVector{7, Float64}, qp::SVector{7, Float64})
gravity!(q::SVector{7, Float64}, G::MVector{7, Float64})
gravity(q::SVector{7, Float64})
```