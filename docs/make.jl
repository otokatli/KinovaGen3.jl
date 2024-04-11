push!(LOAD_PATH,"../src/")

using Documenter
using KinovaGen3
using StaticArrays

makedocs(sitename="KinovaGen3.jl Documentation")

deploydocs(
    repo = "github.com/otokatli/KinovaGen3.jl.git",
)