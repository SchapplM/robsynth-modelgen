% Return the transformation matrices between Floating Base Base Parameter Vector 
% and inertial parameter vector for
% %RN%
% These Matrices can be used to transform between PV2 and MPV
%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% pkin [%NKP%x1]
%   kinematic parameters
% 
% Output:
% K [%NMPVFLOATB%x(%NJ%*10)]
%   matrix for linear conversion from inertial parameters to base parameters, [Sousa2014] equ. (38)
% K_d [%NMPVFLOATB%x(%NMPVFLOATB%-%NJ%*10)]
%   matrix for the influence of linear dependant parameters
% P_b [(%NJ%*10)x%NMPVFLOATB%]
%   Permutation matrix for linear independant inertial parameters, [Sousa2014] equ. (33)
% P_d [(%NJ%*10)x(%NMPVFLOATB%-%NJ%*10)]
%   Permutation matrix for linear dependant inertial parameters, [Sousa2014] equ. (33)
%
% Source:
% [Sousa2014] Sousa, C. D. and Cortesao, R.: Physical feasibility of robot
% base inertial parameter identification: A linear matrix inequality approach (2014)

% %VERSIONINFO%

function [K, K_d, P_b, P_d] = %FN%(pkin)

