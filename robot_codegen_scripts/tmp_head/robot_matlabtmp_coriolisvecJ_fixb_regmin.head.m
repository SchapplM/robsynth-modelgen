% Calculate minimal parameter regressor of coriolis joint torque vector for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_QJD%
% %INPUT_PKIN%
% 
% Output:
% tauc_reg [%NQJ%x%NMPVFIXB%]
%   minimal parameter regressor of coriolis joint torque vector

% %VERSIONINFO%

function tauc_reg = %FN%(qJ, qJD, ...
  pkin)
