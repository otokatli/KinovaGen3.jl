var documenterSearchIndex = {"docs":
[{"location":"#KinovaGen3.jl","page":"KinovaGen3.jl","title":"KinovaGen3.jl","text":"","category":"section"},{"location":"","page":"KinovaGen3.jl","title":"KinovaGen3.jl","text":"CurrentModule = KinovaGen3","category":"page"},{"location":"#Summary","page":"KinovaGen3.jl","title":"Summary","text":"","category":"section"},{"location":"","page":"KinovaGen3.jl","title":"KinovaGen3.jl","text":"This package provides the analytical kinematic and dynamic modelling of the Kinova Gen3 industrial robot.","category":"page"},{"location":"","page":"KinovaGen3.jl","title":"KinovaGen3.jl","text":"The mechanical properties of the robot such as the link lengths, masses, inertia are obtained from the Kinova Gen3 manual.","category":"page"},{"location":"","page":"KinovaGen3.jl","title":"KinovaGen3.jl","text":"In the kinematical model, the reference frames for each link of the robot is selected in accordance with the manual. See page 193 of the manual for the reference frames.","category":"page"},{"location":"","page":"KinovaGen3.jl","title":"KinovaGen3.jl","text":"The aim of this package is to achieve an implementation level without making dynamic allocations when a function of the module is called. However, not all functions of the modules manages to prevent dynamic memory allocation.","category":"page"},{"location":"#Kinematic-Model","page":"KinovaGen3.jl","title":"Kinematic Model","text":"","category":"section"},{"location":"#Position-level-Kinematics","page":"KinovaGen3.jl","title":"Position-level Kinematics","text":"","category":"section"},{"location":"","page":"KinovaGen3.jl","title":"KinovaGen3.jl","text":"forward_kinematics!(q::SVector{7, Float64}, x::MVector{3, Float64}, R::MMatrix{3, 3, Float64})\nforward_kinematics(q::SVector{7, Float64})","category":"page"},{"location":"#KinovaGen3.forward_kinematics!-Tuple{SVector{7, Float64}, MVector{3, Float64}, MMatrix{3, 3, Float64, L} where L}","page":"KinovaGen3.jl","title":"KinovaGen3.forward_kinematics!","text":"forward_kinematics!(q, x, R)\n\nCalculate the position level forward kinematics for Kinova Gen3 robot.\n\nThis function does not perform dynamic memory allocation.\n\nArguments\n\nq::SVector{7, Float64}: the joint angles of the robot [rad]\nx::MVector{3, Float64}: the end-effector position of the robot [m]\nR::MMatrix{3, 3, Float64}: the end-effector orientation of the robot []\n\nExamples\n\njulia> q = SVector(0.0, π / 6, π / 3, π / 2, 0.0, π / 4, 0.0)\njulia> x = zeros(MVector{3})\njulia> R = zeros(MMatrix{3, 3})\njulia> forward_kinematics!(q, x, R)\n([0.32874611223407973, -0.393073312842223, 0.44415411959033596], [-0.6597396084411711 0.75 -0.047367172745376274; 0.6123724356957945 0.5000000000000001 -0.6123724356957946; -0.4355957403991577 -0.43301270189221924 -0.7891491309924313])\n\n\n\n\n\n","category":"method"},{"location":"#KinovaGen3.forward_kinematics-Tuple{SVector{7, Float64}}","page":"KinovaGen3.jl","title":"KinovaGen3.forward_kinematics","text":"forward_kinematics(q)\n\nCalculate the position level forward kinematics for Kinova Gen3 robot.\n\nArguments\n\nq::SVector{7, Float64}: the joint angles of the robot [rad]\n\nExamples\n\njulia> q = SVector(0.0, π / 6, π / 3, π / 2, 0.0, π / 4, 0.0)\njulia> forward_kinematics(q)\n([0.32874611223407973, -0.393073312842223, 0.44415411959033596], [-0.6597396084411711 0.75 -0.047367172745376274; 0.6123724356957945 0.5000000000000001 -0.6123724356957946; -0.4355957403991577 -0.43301270189221924 -0.7891491309924313])\n\n\n\n\n\n","category":"method"},{"location":"#Velocity-level-Kinematics","page":"KinovaGen3.jl","title":"Velocity-level Kinematics","text":"","category":"section"},{"location":"","page":"KinovaGen3.jl","title":"KinovaGen3.jl","text":"jacobian!(q::SVector{7, Float64}, J::MMatrix{6, 7, Float64})\njacobian(q::SVector{7, Float64})\ninverse_kinematics!(q::SVector{7, Float64}, xp::SVector{6, Float64}, qp::MVector{7, Float64}, J::MMatrix{6, 7, Float64})\ninverse_kinematics(q::SVector{7, Float64}, xp::SVector{6, Float64})","category":"page"},{"location":"#KinovaGen3.jacobian!-Tuple{SVector{7, Float64}, MMatrix{6, 7, Float64, L} where L}","page":"KinovaGen3.jl","title":"KinovaGen3.jacobian!","text":"jacobian!(q, J)\n\nCalculate the velocity level forward kinematics for Kinova Gen3 robot.\n\nThis function does not perform dynamic memory allocation.\n\nArguments\n\nq::SVector{7, Float64}: the joint angles of the robot [rad]\nJ::MMatrix{6, 7, Float64}: the geometric Jacobian of the robot []\n\nExamples\n\njulia> q = SVector(0.0, π / 6, π / 3, π / 2, 0.0, π / 4, 0.0)\njulia> J = zeros(MMatrix{6, 7})\njulia> jacobian!(q, J)\n6×7 MMatrix{6, 7, Float64, 42} with indices SOneTo(6)×SOneTo(7):\n -0.393073   0.159344  -0.330236  -0.26764   -0.0886181  -0.11046    0.0\n -0.328746   0.0       -0.20503    0.10253   -0.0591954   0.10253    0.0\n  0.0       -0.328746   0.190662  -0.345176   0.051568   -0.0729318  0.0\n  0.0        0.0       -0.5        0.75      -0.433013    0.75       0.0473672\n  0.0        1.0        0.0        0.5        0.866025    0.5        0.612372\n -1.0        0.0       -0.866025  -0.433013   0.25       -0.433013   0.789149\n\n\n\n\n\n","category":"method"},{"location":"#KinovaGen3.jacobian-Tuple{SVector{7, Float64}}","page":"KinovaGen3.jl","title":"KinovaGen3.jacobian","text":"jacobian(q)\n\nCalculate the velocity level forward kinematics for Kinova Gen3 robot.\n\nArguments\n\nq::SVector{7, Float64}: the joint angles of the robot [rad]\n\nExamples\n\njulia> q = SVector(0.0, π / 6, π / 3, π / 2, 0.0, π / 4, 0.0)\njulia> jacobian(q)\n6×7 MMatrix{6, 7, Float64, 42} with indices SOneTo(6)×SOneTo(7):\n -0.393073   0.159344  -0.330236  -0.26764   -0.0886181  -0.11046    0.0\n -0.328746   0.0       -0.20503    0.10253   -0.0591954   0.10253    0.0\n  0.0       -0.328746   0.190662  -0.345176   0.051568   -0.0729318  0.0\n  0.0        0.0       -0.5        0.75      -0.433013    0.75       0.0473672\n  0.0        1.0        0.0        0.5        0.866025    0.5        0.612372\n -1.0        0.0       -0.866025  -0.433013   0.25       -0.433013   0.789149\n\n\n\n\n\n","category":"method"},{"location":"#KinovaGen3.inverse_kinematics!-Tuple{SVector{7, Float64}, SVector{6, Float64}, MVector{7, Float64}, MMatrix{6, 7, Float64, L} where L}","page":"KinovaGen3.jl","title":"KinovaGen3.inverse_kinematics!","text":"inverse_kinematics!(q, xp, qp, J)\n\nCalculate the velocity level inverse kinematics for Kinova Gen3 robot.\n\nThis method reduces the number of allocations\n\nNote that LinearAlgebra.pinv cause allocations during runtime\n\nArguments\n\nq::SVector{7, Float64}: the joint angles of the robot [rad]\nxp::SVector{6, Float64}: the end-effector velocities of the robot [m/s]\nqp::MVector{6, Float64}: the joint space velocities of the robot [rad/s]\nJ::MMatrix{6, 7, Float64}: the Jacobian matrix of the robot []\n\nExamples\n\njulia> q = SVector(0.0, π / 6, π / 3, π / 2, 0.0, π / 4, 0.0)\njulia> xp = SVector(5.519897654292758, 3.8454568904118693, 5.400775529650293, 0.05767137155933982, 0.09150292411182576, 0.33940753291008463)\njulia> qp = zeros(MVector{7})\njulia> J = zeros(MMatrix{6, 7})\njulia> inverse_kinematics!(q, xp, qp, J)\n7-element MVector{7, Float64} with indices SOneTo(7):\n -13.741946912596518\n   0.24414409836241704\n   5.390106099987782\n -17.3961161502307\n   2.6801993763554823\n  23.166518880789784\n  -8.751150401752152\n\n\n\n\n\n","category":"method"},{"location":"#KinovaGen3.inverse_kinematics-Tuple{SVector{7, Float64}, SVector{6, Float64}}","page":"KinovaGen3.jl","title":"KinovaGen3.inverse_kinematics","text":"inverse_kinematics(q, xp)\n\nCalculate the velocity level inverse kinematics for Kinova Gen3 robot.\n\nArguments\n\nq::SVector{7, Float64}: the joint angles of the robot [rad]\nxp::SVector{6, Float64}: the end-effector velocities of the robot [m/s]\n\nExamples\n\njulia> q = SVector(0.0, π / 6, π / 3, π / 2, 0.0, π / 4, 0.0)\njulia> xp = SVector(5.519897654292758, 3.8454568904118693, 5.400775529650293, 0.05767137155933982, 0.09150292411182576, 0.33940753291008463)\njulia> inverse_kinematics(q, xp)\n7-element MVector{7, Float64} with indices SOneTo(7):\n -13.741946912596518\n   0.24414409836241704\n   5.390106099987782\n -17.3961161502307\n   2.6801993763554823\n  23.166518880789784\n  -8.751150401752152\n\n\n\n\n\n","category":"method"},{"location":"#Dynamics-Model","page":"KinovaGen3.jl","title":"Dynamics Model","text":"","category":"section"},{"location":"","page":"KinovaGen3.jl","title":"KinovaGen3.jl","text":"mass_matrix!(q::SVector{7, Float64}, M::MMatrix{7, 7, Float64})\nmass_matrix(q::SVector{7, Float64})\ncoriolis!(q::SVector{7, Float64}, qp::SVector{7, Float64}, C::MVector{7, Float64})\ncoriolis(q::SVector{7, Float64}, qp::SVector{7, Float64})\ngravity!(q::SVector{7, Float64}, G::MVector{7, Float64})\ngravity(q::SVector{7, Float64})","category":"page"},{"location":"#KinovaGen3.mass_matrix!-Tuple{SVector{7, Float64}, MMatrix{7, 7, Float64, L} where L}","page":"KinovaGen3.jl","title":"KinovaGen3.mass_matrix!","text":"mass_matrix!(q, M)\n\nCalculate the mass matrix of Kinova Gen3 robot for a given configuration.\n\nThis function reduces the number of dynamic memory allocation.\n\nArguments\n\nq::SVector{7, Float64}: the joint angles of the robot [rad]\nM::MMatrix{7, 7, Float64}: the mass matrix of the robot []\n\nExamples\n\njulia> q = SVector(0.0, π / 6, π / 3, π / 2, 0.0, π / 4, 0.0)\njulia> M = zeros(MMatrix{7, 7})\njulia> mass_matrix!(q, M)\n7×7 MMatrix{7, 7, Float64, 49} with indices SOneTo(7)×SOneTo(7):\n  0.494054     -0.172401      0.26987      0.097991      0.030743     0.00400175   -0.000391656\n -0.172401      0.652499     -0.235912     0.0970044    -0.0183927    0.00189615    0.000522066\n  0.26987      -0.235912      0.234615     0.000851294   0.0278451    0.00085165   -0.000479402\n  0.097991      0.0970044     0.000851294  0.238603      0.00116619   0.0346251     0.00192549\n  0.030743     -0.0183927     0.0278451    0.00116619    0.00750583   3.56828e-5    0.000463883\n  0.00400175    0.00189615    0.00085165   0.0346251     3.56828e-5   0.0126349     0.000656714\n -0.000391656   0.000522066  -0.000479402  0.00192549    0.000463883  0.000656714   0.00067412\n\n\n\n\n\n","category":"method"},{"location":"#KinovaGen3.mass_matrix-Tuple{SVector{7, Float64}}","page":"KinovaGen3.jl","title":"KinovaGen3.mass_matrix","text":"mass_matrix(q)\n\nCalculate the mass matrix of Kinova Gen3 robot for a given configuration.\n\nArguments\n\nq::SVector{7, Float64}: the joint angles of the robot [rad]\n\nExamples\n\njulia> q = SVector(0.0, π / 6, π / 3, π / 2, 0.0, π / 4, 0.0)\njulia> mass_matrix(q)\n7×7 MMatrix{7, 7, Float64, 49} with indices SOneTo(7)×SOneTo(7):\n  0.494054     -0.172401      0.26987      0.097991      0.030743     0.00400175   -0.000391656\n -0.172401      0.652499     -0.235912     0.0970044    -0.0183927    0.00189615    0.000522066\n  0.26987      -0.235912      0.234615     0.000851294   0.0278451    0.00085165   -0.000479402\n  0.097991      0.0970044     0.000851294  0.238603      0.00116619   0.0346251     0.00192549\n  0.030743     -0.0183927     0.0278451    0.00116619    0.00750583   3.56828e-5    0.000463883\n  0.00400175    0.00189615    0.00085165   0.0346251     3.56828e-5   0.0126349     0.000656714\n -0.000391656   0.000522066  -0.000479402  0.00192549    0.000463883  0.000656714   0.00067412\n\n\n\n\n\n","category":"method"},{"location":"#KinovaGen3.coriolis!-Tuple{SVector{7, Float64}, SVector{7, Float64}, MVector{7, Float64}}","page":"KinovaGen3.jl","title":"KinovaGen3.coriolis!","text":"coriolis!(q, qp, C)\n\nCalculate the Coriolis term, C(q, qp)qp, of Kinova Gen3 robot for a given configuration.\n\nThis function reduces the number of dynamic memory allocation.\n\nArguments\n\nq::SVector{7, Float64}: the joint angles of the robot [rad]\nqp::SVector{7, Float64}: the joint velocities of the robot [rad/s]\nC::MVector{7, Float64}: the mass matrix of the robot []\n\nExamples\n\njulia> q = SVector(0.0, π / 6, π / 3, π / 2, 0.0, π / 4, 0.0)\njulia> qp = SVector(0.08123992859372045, 0.1594522268081302, 0.14487502589784196, 0.09843558598385844, 0.06467638628492739, 0.05374794957035123, 0.07384203846053861)\njulia> C = zeros(MVector{7})\njulia> coriolis!(q, qp, C)\n7-element MVector{7, Float64} with indices SOneTo(7):\n  0.004668503395502678\n -0.021228409144909945\n  0.009371757499649963\n -0.000447318871758321\n  0.0008351413947819465\n  0.001688511069591081\n  0.00010500683267616118\n\n\n\n\n\n","category":"method"},{"location":"#KinovaGen3.coriolis-Tuple{SVector{7, Float64}, SVector{7, Float64}}","page":"KinovaGen3.jl","title":"KinovaGen3.coriolis","text":"coriolis(q, qp)\n\nCalculate the Coriolis term, C(q, qp)qp, of Kinova Gen3 robot for a given configuration.\n\nArguments\n\nq::SVector{7, Float64}: the joint angles of the robot [rad]\nqp::SVector{7, Float64}: the joint velocities of the robot [rad/s]\n\nExamples\n\njulia> q = SVector(0.0, π / 6, π / 3, π / 2, 0.0, π / 4, 0.0)\njulia> qp = SVector(0.08123992859372045, 0.1594522268081302, 0.14487502589784196, 0.09843558598385844, 0.06467638628492739, 0.05374794957035123, 0.07384203846053861)\njulia> coriolis(q, qp)\n7-element MVector{7, Float64} with indices SOneTo(7):\n  0.004668503395502678\n -0.021228409144909945\n  0.009371757499649963\n -0.000447318871758321\n  0.0008351413947819465\n  0.001688511069591081\n  0.00010500683267616118\n\n\n\n\n\n","category":"method"},{"location":"#KinovaGen3.gravity!-Tuple{SVector{7, Float64}, MVector{7, Float64}}","page":"KinovaGen3.jl","title":"KinovaGen3.gravity!","text":"gravity!(q, G)\n\nCalculate the gravity term of Kinova Gen3 robot for a given configuration.\n\nThis function reduces the number of dynamic memory allocation.\n\nArguments\n\nq::SVector{7, Float64}: the joint angles of the robot [rad]\nG::MVector{7, Float64}: the gravity term of the robot []\n\nExamples\n\njulia> q = SVector(0.0, π / 6, π / 3, π / 2, 0.0, π / 4, 0.0)\njulia> G = zeros(MVector{7})\njulia> gravity!(q, G)\n7-element MVector{7, Float64} with indices SOneTo(7):\n   1.7862417274472353e-16\n -10.576350989174017\n   3.0274799819651212\n  -5.734529044767631\n   0.2514842083025938\n  -0.42307721718230645\n  -0.024979720551009294\n\n\n\n\n\n","category":"method"},{"location":"#KinovaGen3.gravity-Tuple{SVector{7, Float64}}","page":"KinovaGen3.jl","title":"KinovaGen3.gravity","text":"gravity(q)\n\nCalculate the gravity term of Kinova Gen3 robot for a given configuration.\n\nArguments\n\nq::SVector{7, Float64}: the joint angles of the robot [rad]\n\nExamples\n\njulia> q = SVector(0.0, π / 6, π / 3, π / 2, 0.0, π / 4, 0.0)\njulia> gravity(q)\n7-element MVector{7, Float64} with indices SOneTo(7):\n   1.7862417274472353e-16\n -10.576350989174017\n   3.0274799819651212\n  -5.734529044767631\n   0.2514842083025938\n  -0.42307721718230645\n  -0.024979720551009294\n\n\n\n\n\n","category":"method"}]
}