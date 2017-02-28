% Calculate inertial parameter regressor for vector of inverse dynamics joint torques for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) [rad]
% qD [%NQJ%x1]
%   Generalized velocities (joint velocities) [rad/s]
% qD [%NQJ%x1]
%   Generalized accelerations (joint accelerations) [rad/s]
% r_base [3x1]
%   Base position in world frame
% phi_base [3x1]
%   Base orientation in world frame. Expressed with RPY Euler angles (xyz)
% xD_base [6x1]
%   time derivative of r_base and phi_base
% xDD_base [6x1]
%   second time derivative of r_base and phi_base
% g [3x1]
%   gravitation vector in world frame [m/s^2]
% pkin [%NKP%x1]
%   kinematic parameters
% 
% Output:
% tau_reg [%NQJ%x(10*%NL%)]
%   inertial parameter regressor for joint torques of inverse dynamics
%   (contains inertial, gravitational coriolis and centrifugal forces)

% %VERSIONINFO%

function tau_reg = %FN%(q, qD, qDD, phi_base, xD_base, xDD_base, g, pkin)