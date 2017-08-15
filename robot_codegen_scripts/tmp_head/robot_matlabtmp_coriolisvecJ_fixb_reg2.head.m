% Calculate inertial parameters regressor of coriolis joint torque vector for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_Q%
% %INPUT_QD%
% %INPUT_PKIN%
% 
% Output:
% taug_reg [%NQJ%x(%NQJ%*10)]
%   inertial parameter regressor of coriolis joint torque vector

% %VERSIONINFO%

function tauc_reg = %FN%(q, qD, pkin)
