% Calculate joint inertia matrix for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) [rad]
% pkin [%NKP%x1]
%   kinematic parameters
% m_num_mdh, rSges_num_mdh, Icges_num_mdh [%NL%x1]
%   dynamic parameters (parameter set 1: center of mass and inertia about center of mass)
% 
% Output:
% Mq [%NQJ%x%NQJ%]
%   inertia matrix

% %VERSIONINFO%

function Mq = %FN%(q, ...
  pkin, m_num, rSges_num_mdh, Icges_num_mdh)
