% Calculate forward kinematics (homogenous transformation matrices) for fixed-base
% %RN% (for one body)
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% link_index [1x1 uint8]
%   index of the body frame to be returned (0=base).
% %INPUT_PKIN%
% 
% Output:
% Tc_mdh [4x4]
%   homogenous transformation matrices for the body frame of "link_index"

% %VERSIONINFO%

function Tc_mdh = %FN%(qJ, link_index, pkin)
