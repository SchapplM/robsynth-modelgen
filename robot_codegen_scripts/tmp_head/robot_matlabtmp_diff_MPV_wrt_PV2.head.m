% Return the Partial differentiation of Fixed Base Base Parameter Vector 
% with respect to Parameter Vector 2 for
% %RN%
% This Matrix can be used to transform between PV2 and MPV
%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% pkin [%NKP%x1]
%   kinematic parameters
% m_num_mdh [%NL%x1], mrSges_num_mdh [%NL%x3], Ifges_num_mdh [%NL%x6]
%   dynamic parameters (parameter set 2: first moment and inertia about link frame origin)
% 
% Output:
% dMPVdPV2 [%NMPVFIXB%x(%NJ%*10)]
%   matrix for linear conversion from inertial parameters to base parameters

% %VERSIONINFO%

function dMPVdPV2 = %FN%(pkin)

