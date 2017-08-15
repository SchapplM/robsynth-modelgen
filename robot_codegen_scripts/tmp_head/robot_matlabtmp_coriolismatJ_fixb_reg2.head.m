% Calculate inertial parameters regressor of coriolis matrix for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_Q%
% %INPUT_QD%
% %INPUT_PKIN%
% 
% Output:
% cmat_reg [(%NQJ%*%NQJ%)x(%NQJ%*10)]
%   inertial parameter regressor of coriolis matrix

% %VERSIONINFO%

function cmat_reg = %FN%(qJ, qJD, pkin)
