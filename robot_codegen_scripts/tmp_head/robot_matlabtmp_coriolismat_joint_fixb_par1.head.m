% Calculate matrix of centrifugal and coriolis load on the joints for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) [rad]
% qD [%NQJ%x1]
%   Generalized velocities (joint velocities) [rad/s]
% a_mdh, d_mdh, q_offset_mdh [%NJ%x1]
%   kinematic parameters
% m_num_mdh, rSges_num_mdh, Icges_num_mdh [%NL%x1]
%   dynamic parameters (parameter set 1: center of mass and inertia about center of mass)
% 
% Output:
% Cq [%NJ%x%NJ%]
%   matrix of coriolis and centrifugal joint torques.
%   Gives coriolis joint torques when multiplied with joint velocities

function Cq = %FN%(q, qD, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh, m_num, rSges_num_mdh, Icges_num_mdh%KCPARG%)
