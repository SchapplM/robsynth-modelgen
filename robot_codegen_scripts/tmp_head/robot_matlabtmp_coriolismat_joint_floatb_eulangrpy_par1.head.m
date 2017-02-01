% Calculate matrix of centrifugal and coriolis load on the joints for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) [rad]
% qD [%NQJ%x1]
%   Generalized velocities (joint velocities) [rad/s]
% r_base [3x1]
%   Base position in world frame
% phi_base [3x1]
%   Base orientation in world frame. Expressed with RPY Euler angles (xyz)
% xD_base [6x1]
%   time derivative of r_base and phi_base
% pkin [%NKP%x1]
%   kinematic parameters
% m_num_mdh, rSges_num_mdh, Icges_num_mdh [%NL%x1]
%   dynamic parameters (parameter set 1: center of mass and inertia about center of mass)
% 
% Output:
% Cq [%NQJ%x(6+%NQJ%)]
%   matrix of coriolis and centrifugal joint torques
%   Gives coriolis joint torques when multiplied with base and joint velocities

% %VERSIONINFO%

function Cq = %FN%(q, qD, phi_base, xD_base, ...
  pkin, m_num, rSges_num_mdh, Icges_num_mdh)
