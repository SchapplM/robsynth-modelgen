% Calculate matrix of centrifugal and coriolis load on the base and joints for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_QJD%
% %INPUT_PHIB%
% %INPUT_RB%
% %INPUT_XDB%

% %INPUT_PKIN%
% %INPUT_M%
% %INPUT_MR%
% %INPUT_IF%
% 
% Output:
% Cq [(6+%NQJ%)x(6+%NQJ%)]
%   matrix of coriolis and centrifugal generalized forces
%   Gives coriolis base forces/torques and joint torques when multiplied with base and joint velocities

% %VERSIONINFO%

function Cq = %FN%(qJ, qJD, phi_base, xD_base, ...
  pkin, m_num, mrSges_num_mdh, Ifges_num_mdh)
