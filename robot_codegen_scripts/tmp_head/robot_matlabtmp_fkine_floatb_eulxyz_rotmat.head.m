% Calculate forward kinematics (homogenous transformation matrices) for floating-base
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_RB%
% %INPUT_PHIB%
% %INPUT_PKIN%
% 
% Output:
% Tc_mdh [4x4x(%NJ%+1)]
%   homogenous transformation matrices for each (body) frame (MDH)
%   1:  world -> mdh base (link 1)
%   2:  world -> mdh link 2 (first articulated link)
%   ...
%   %NL%:  world -> mdh link (%NL%-1) (last articulated link)
%   ...
%   %NJ%+1:  mdh base (link 0) -> mdh frame (%NJ%-1)
% Tc_stack [(%NJ%+1)*3 x 4]
%   stacked matrices from Tc_mdh into one 2D array, last row left out.
%   Last row only contains [0 0 0 1].

% %VERSIONINFO%

function [Tc_mdh, Tc_stack] = %FN%(qJ, r_base, phi_base, pkin)
