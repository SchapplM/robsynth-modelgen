% Calculate inertia matrix time derivative for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) [rad]
% qD [%NQJ%x1]
%   Generalized velocities (joint velocities) [rad/s]
% phi_base [3x1]
%   Base orientation in world frame. Expressed with RPY Euler angles (xyz)
% xD_base [6x1]
%   time derivative of 
%   r_base (3x1 Base position in world frame) and 
%   phi_base (3x1)
% a_mdh, d_mdh, q_offset_mdh [%NJ%x1]
%   kinematic parameters
% m_num_mdh, rSges_num_mdh, Icges_num_mdh [%NL%x1]
%   dynamic parameters (parameter set 1: center of mass and inertia about center of mass)
% 
% Output:
% MD [(6+%NJ%)x(6+%NJ%)]
%   full time derivative of inertia matrix (for base and joint dynamics)

function MD = %FN%(q, qD, phi_base, xD_base, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh, m_num, rSges_num_mdh, Icges_num_mdh)
