% Calculate minimal parameter regressor of inverse dynamics forces for
% %RN%
% Use Code from Maple symbolic Code Generation
%
% Input:
% %INPUT_XP%
% %INPUT_XDP%
% %INPUT_XDDP%
% %INPUT_QJ_P%
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% %INPUT_LEGFRAME%
% %INPUT_PKIN%
% %INPUT_KOPPEL%

% Output:
% tau_reg [%N_XP%x%NMPVPARA%]
%   minimal parameter regressor of inverse dynamics force vector
%   in task space

% %VERSIONINFO%

function tau_reg = %FN%(xP, xDP, xDDP, qJ, g, legFrame, ...
  koppelP, pkin)
