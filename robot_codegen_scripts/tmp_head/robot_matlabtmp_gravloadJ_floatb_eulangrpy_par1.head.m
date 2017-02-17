% Calculate Gravitation load on the joints for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) [rad]
% phi_base [3x1]
%   Base orientation in world frame. Expressed with RPY Euler angles (xyz)
% g [3x1]
%   gravitation vector in world frame [m/s^2]
% pkin [%NKP%x1]
%   kinematic parameters
% m_num_mdh, rSges_num_mdh [%NL%x1]
%   dynamic parameters
% 
% Output:
% taug [%NQJ%x1]
%   joint torques required to compensate gravitation load

% %VERSIONINFO%

function taug = %FN%(q, phi_base, g, ...
  pkin, m_num, rSges_num_mdh)
