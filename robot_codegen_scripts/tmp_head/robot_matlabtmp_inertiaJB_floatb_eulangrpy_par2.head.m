% Calculate joint-base inertia matrix for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) [rad]
% pkin [%NKP%x1]
%   kinematic parameters
% m_num_mdh, mrSges_num_mdh, Ifges_num_mdh [%NL%x1]
%   dynamic parameters (parameter set 2: first moment and inertia about link frame origin)
% 
% Output:
% Mqb [%NQJ%x6]
%   joint-base inertia matrix (gives inertial forces on the joins from base acceleration)

% %VERSIONINFO%

function Mqb = %FN%(q, phi_base, ...
  pkin, m_num, mrSges_num_mdh, Ifges_num_mdh)
