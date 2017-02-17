% Calculate base parameter regressor of matrix of centrifugal and coriolis load on the base and joints for
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
% 
% Output:
% cmat_reg [((6+%NQJ%)*(6+%NQJ%))x%NMPVFLOATB%]
%   base parameter regressor of matrix of coriolis and centrifugal generalized forces
%   Gives coriolis base forces/torques and joint torques when multiplied with base and joint velocities

% %VERSIONINFO%

function Cq = %FN%(q, qD, phi_base, xD_base, pkin)
