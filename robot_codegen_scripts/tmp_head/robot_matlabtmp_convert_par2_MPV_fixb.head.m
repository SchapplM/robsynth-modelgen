% Return the minimum parameter vector for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% pkin [%NKP%x1]
%   kinematic parameters
% m_num_mdh, mrSges_num_mdh, Ifges_num_mdh [%NL%x1]
%   dynamic parameters (parameter set 2: first moment and inertia about link frame origin)
% 
% Output:
% MPV [%NMPVFIXB%x1]
%   base parameter vector (minimal parameter vector)

% %VERSIONINFO%

function MPV = %FN%(pkin, m_num, mrSges_num_mdh, Ifges_num_mdh)

