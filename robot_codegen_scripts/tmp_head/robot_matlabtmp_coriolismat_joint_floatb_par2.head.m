% Calculate matrix of centrifugal and coriolis load on the joints for
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
% Cq [%NJ%x(6+%NJ%)]
%   matrix of coriolis and centrifugal joint torques
%   Gives coriolis joint torques when multiplied with base and joint velocities

function Cq = %RN%_coriolisvec_joint_floatb_sym_lag_varpar_par2(q, qD, V_base, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh, m_num, mrSges_num_mdh, Ifges_num_mdh)
