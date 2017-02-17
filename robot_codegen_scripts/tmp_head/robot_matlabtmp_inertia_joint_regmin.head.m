% Calculate minimal parameter regressor of joint inertia matrix for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) [rad]
% pkin [%NKP%x1]
%   kinematic parameters
% 
% Output:
% MM_reg [((%NQJ%+1)*%NQJ%/2)x%NMPV%]
%   minimal parameter regressor of joint inertia matrix
%   (only lower left triangular matrix (including diagonal) due to symmetry

% %VERSIONINFO%

function MM_reg = %FN%(q, pkin)
