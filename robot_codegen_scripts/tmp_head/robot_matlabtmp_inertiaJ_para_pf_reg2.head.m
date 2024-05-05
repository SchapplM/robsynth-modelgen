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
% tau_reg [(%N_XP%*%N_XP)%x(10*%NL%)]
%   inertial parameter regressor of inertia matrix for parallel robot
%   in task space

% %VERSIONINFO%

function tau_reg = %FN%(xP, qJ, legFrame, ...
  koppelP, pkin)
