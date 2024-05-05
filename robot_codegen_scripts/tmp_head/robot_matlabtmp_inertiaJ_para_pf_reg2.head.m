% Calculate inertial parameter regressor of inertia matrix for parallel robot
% %RN%
% Use Code from Maple symbolic Code Generation
%
% Input:
% %INPUT_XP%
% %INPUT_QJ_P%
% %INPUT_LEGFRAME%
% %INPUT_PKIN%
% %INPUT_KOPPEL%

% Output:
% tau_reg [(%N_XP%*%N_XP)%x(10*(%NQJ_P%+1))]
%   inertial parameter regressor of inertia matrix for parallel robot
%   in task space
%   The columns refer to 10 parameters each for the bodies of the first 
%   %NQJ_P% joint DoF and the platform.

% %VERSIONINFO%

function tau_reg = %FN%(xP, qJ, legFrame, ...
  koppelP, pkin)
