% Calculate matrix of centrifugal and coriolis load on the base and joints for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NJ%x1]
%   Joint Angles [rad]
% qD [%NJ%x1]
%   Joint Velocities [rad/s]
% r_base [3x1]
%   Base position in world frame
% phi_base [3x1]
%   Base orientation in world frame. Expressed with RPY Euler angles (xyz)
% xD_base [6x1]
%   time derivative of r_base and phi_base
% a_mdh, d_mdh, q_offset_mdh [%NJ%x1]
%   kinematic parameters
% m_num_mdh, mrSges_num_mdh, Ifges_num_mdh [%NL%x1]
%   dynamic parameters (parameter set 2: first moment and inertia about link frame origin)
% 
% Output:
% Cq [(6+%NJ%)x(6+%NJ%)]
%   matrix of coriolis and centrifugal generalized forces
%   Gives coriolis base forces/torques and joint torques when multiplied with base and joint velocities

function Cq = %FN%(q, qD, phi_base, xD_base, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh, m_num, mrSges_num_mdh, Ifges_num_mdh)
