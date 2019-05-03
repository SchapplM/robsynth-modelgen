% Calculate minimal parameter regressor of vector of centrifugal and coriolis load for parallel robot
% %RN%
% Use Code from Maple symbolic Code Generation
%
% Input:
% %INPUT_XP%
% %INPUT_XDP%
% %INPUT_QJ_P%
% %INPUT_LEGFRAME%
% %INPUT_PKIN%
% %INPUT_KOPPEL%

% Output:
% tau_reg [%N_XP%x%NMPVPARA%]
%   minimal parameter regressor of vector of coriolis and centrifugal joint torques
%   in task space

% %VERSIONINFO%

function tau_reg = %FN%(xP, xDP, qJ, legFrame, ...
  koppelP, pkin)
