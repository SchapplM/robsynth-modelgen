% Calculate minimal parameter regressor of Coriolis joint torque vector for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_QJD%
% %INPUT_PKIN%
% %INPUT_MDPFIXB%
% 
% Output:
% tauc [%NQJ%x1]
%   joint torques required to compensate Coriolis and centrifugal load

% %VERSIONINFO%

function tauc = %FN%(qJ, qJD, pkin, MDP)
