% Calculate forward kinematics (homogenous transformation matrices) for fixed-base
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_PKIN%
% 
% Output:
% T_c_mdh [4x4x(%NJ%+1)]
%   homogenous transformation matrices for each (body) frame (MDH)
%   1:  mdh base (link 0) -> mdh base link 0 (unit matrix, no information)
%   ...
%   %NL%:  mdh base (link 0) -> mdh frame (%NL%-1), link (%NL%-1)
%   ...
%   %NJ%+1:  mdh base (link 0) -> mdh frame (%NJ%)

% %VERSIONINFO%

function T_c_mdh = %FN%(qJ, pkin)
