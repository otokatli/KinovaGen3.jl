using KinovaGen3
using StaticArrays
using Test

@testset "ForwardKinematics" begin
    # Test configurations
    q = SA[103.3 7.3 15.7 -113.1 232.7 120.3 271;
           100.5 45 311.2 147.7 107.6 120.3 146.2;
           349.9 75.8 217 15.5 107.6 250.3 150.6;
           104.9 -20.2 239 -1.8 224.4 50.1 229.5;
           33.2 85.2 30.2 -123.7 93.9 70.5 154.5;
           231.4 53.6 233.3 20.7 238.9 257.5 255.1;
           296.8 -80.1 216 99.5 207.9 60.9 284.9;
           272.7 -42.8 133 77.6 257.7 320.4 147.2;
           23.1 85.6 246.3 -135.7 334.9 52.1 279.6] * π / 180.0

    # The following end-effector positions are obtained from the robot's web interface
    x = SA[0.157 0.076 0.551;
           -0.051 -0.328 0.307;
           0.594 0.052 0.615;
           -0.059 0.328 1.068;
           0.144 -0.102 0.647;
           -0.506 0.279 0.658;
           -0.408 -0.397 0.007;
           0.223 -0.628 0.386;
           0.05 -0.428 0.189]
    
    for (qi, xi) in zip(eachrow(q), eachrow(x))
        x_test, R_test = KinovaGen3.forward_kinematics(qi) 
        @test x_test ≈ xi atol=0.001
    end
end
