% Calculate vector of centrifugal and coriolis load on the joints for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_XP%
% %INPUT_XDP%
% %INPUT_QJ_P%
% %INPUT_LEGFRAME%
% %INPUT_KOPPEL%
% %INPUT_PKIN%
% %INPUT_M_P%
% %INPUT_R_P%
% %INPUT_IC_P%
% 
% Output:
% C [%N_LEGS%x%N_LEGS%]
%   matrix of coriolis and centrifugal joint torques

% %VERSIONINFO%

function C = %FN%(xP, xDP, qJ, legFrame, ...
  koppelP, pkin, m, rSges, Icges)
