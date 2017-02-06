% Calculate minimal parameter regressor of coriolis matrix for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) [rad]
% qD [%NQJ%x1]
%   Generalized velocities (joint velocities) [rad/s]
% pkin [%NKP%x1]
%   kinematic parameters
% 
% Output:
% cmat_reg [(%NQJ%*%NQJ)%x%NMPV%]
%   minimal parameter regressor of coriolis matrix

% %VERSIONINFO%

function cmat_reg = %FN%(q, qD, ...
  pkin)
