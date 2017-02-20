% Calculate forward kinematics (homogenous transformation matrices) for fixed-base
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) (generalized coordinates) [rad]
% pkin [%NKP%x1]
%   kinematic parameters
% 
% Output:
% T_c_mdh [4x4x%NL%]
%   homogenous transformation matrices for each body frame (MDH)
%   1:  mdh base (link 0) -> mdh base link 0 (unit matrix, no information)
%   ...
%   %NL%:  mdh base (link 0) -> mdh link (%NL%-1)

% %VERSIONINFO%

function T_c_mdh = %FN%(q, pkin)
