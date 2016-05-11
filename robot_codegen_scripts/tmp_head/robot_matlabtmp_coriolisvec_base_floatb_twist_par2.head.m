% Calculate vector of centrifugal and coriolis load on the base for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NJ%x1]
%   Joint Angles [rad]
% qD [%NJ%x1]
%   Joint Velocities [rad/s]
% V_base [6x1]
%   Base Velocity (twist: stacked translational and angular velocity) in base frame
% a_mdh, d_mdh, q_offset_mdh [%NJ%x1]
%   kinematic parameters
% m_num_mdh, mrSges_num_mdh, Ifges_num_mdh [%NL%x1]
%   dynamic parameters (parameter set 2: first moment and inertia about link frame origin)
% 
% Output:
% Fc [6x1]
%   base forces and torques required to compensate coriolis and centrifugal load

function Fc = %FN%(q, qD, V_base, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh, m_num, mrSges_num_mdh, Ifges_num_mdh)
