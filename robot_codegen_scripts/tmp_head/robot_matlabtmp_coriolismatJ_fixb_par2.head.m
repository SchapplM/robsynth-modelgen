% Calculate matrix of centrifugal and coriolis load on the joints for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) [rad]
% qD [%NQJ%x1]
%   Generalized velocities (joint velocities) [rad/s]
% pkin [%NKP%x1]
%   kinematic parameters
% m_num_mdh, mrSges_num_mdh, Ifges_num_mdh [%NL%x1]
%   dynamic parameters (parameter set 2: first moment and inertia about link frame origin)
% 
% Output:
% Cq [%NQJ%x%NQJ%]
%   matrix of coriolis and centrifugal joint torques

% %VERSIONINFO%

function Cq = %FN%(q, qD, ...
  pkin, m_num, mrSges_num_mdh, Ifges_num_mdh)