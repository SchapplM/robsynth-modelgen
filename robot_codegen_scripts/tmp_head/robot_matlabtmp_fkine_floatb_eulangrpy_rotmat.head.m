% Calculate matrix of centrifugal and coriolis load on the joints for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NJ%x1]
%   Joint Angles [rad]
% r_base [3x1]
%   Base position in world frame
% phi_base [3x1]
%   Base orientation in world frame. Expressed with RPY Euler angles (xyz)
% a_mdh, d_mdh, q_offset_mdh [%NJ%x1]
%   kinematic parameters
% 
% Output:
% T_c_mdh [4x4x%NL%]
%   homogenious transformation matrices for each body frame (MDH)
%   1:  world -> mdh base (link 1)
%   2:  world -> mdh link 2 (first articulated link)
%   ...
%   %NL%:  world -> mdh link %NL% (last articulated link)

function T_c_mdh = %FN%(q, r_base, phi_base, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh)
