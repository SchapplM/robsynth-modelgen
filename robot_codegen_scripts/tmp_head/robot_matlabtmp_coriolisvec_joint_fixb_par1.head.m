% Calculate vector of centrifugal and coriolis load on the joints for
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
% m_num_mdh, rSges_num_mdh, Icges_num_mdh [%NL%x1]
%   dynamic parameters (parameter set 1: center of mass and inertia about center of mass)
% 
% Output:
% tauc [%NQJ%x1]
%   joint torques required to compensate coriolis and centrifugal load

% %VERSIONINFO%

function tauc = %FN%(q, qD, ...
  pkin, m_num, rSges_num_mdh, Icges_num_mdh)
