% Calculate forward kinematics (homogenous transformation matrices) for fixed-base
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_PKIN%
% 
% Output:
% Tc_mdh [4x4x(%NJ%+1)]
%   homogenous transformation matrices for each (body) frame (MDH)
%   1:  mdh base (link 0) -> mdh base link 0 (unit matrix, no information)
%   ...
%   %NL%:  mdh base (link 0) -> mdh frame (%NL%-1), link (%NL%-1)
%   ...
%   %NJ%+1:  mdh base (link 0) -> mdh frame (%NJ%)
% T_c_stack [(%NJ%+1)*3 x 4]
%   stacked matrices from Tc_mdh into one 2D array, last row left out.
%   Last row only contains [0 0 0 1].

% %VERSIONINFO%

function [Tc_mdh, Tc_stack] = %FN%(qJ, pkin)
