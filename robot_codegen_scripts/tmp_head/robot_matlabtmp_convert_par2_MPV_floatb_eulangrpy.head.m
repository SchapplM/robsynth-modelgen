% Return the minimum parameter vector for
% %RN%
% for eulangrpy floating base model
% Use Code from Maple symbolic Code Generation
% 
% Input:
% pkin [%NKP%x1]
%   kinematic parameters
% m_num_mdh [%NL%x1], mrSges_num_mdh [%NL%x3], Ifges_num_mdh [%NL%x6]
%   dynamic parameters (parameter set 2: first moment and inertia about link frame origin)
% 
% Output:
% MPV [%NMPVFLOATB%x1]
%   base parameter vector (minimal parameter vector)

% %VERSIONINFO%

function MPV = %FN%(pkin, m_num, mrSges_num_mdh, Ifges_num_mdh)
