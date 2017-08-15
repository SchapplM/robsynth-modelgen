% Return the transformation matrices between Fixed Base Base Parameter Vector 
% and inertial parameter vector for
% %RN%
% These Matrices can be used to transform between PV2 and MPV
%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_PKIN%
% 
% Output:
% K [%NMPVFIXB%x(%NJ%*10)]
%   matrix for linear conversion from inertial parameters to base parameters, [Sousa2014] equ. (38)
% K_d [%NMPVFIXB%x(%NJ%*10-%NMPVFIXB%)]
%   matrix for the influence of linear dependant parameters
% P_b [(%NJ%*10)x%NMPVFIXB%]
%   Permutation matrix for linear independant inertial parameters, [Sousa2014] equ. (33)
% P_d [(%NJ%*10)x(%NJ%*10-%NMPVFIXB%)]
%   Permutation matrix for linear dependant inertial parameters, [Sousa2014] equ. (33)
%
% Source:
% [Sousa2014] Sousa, C. D. and Cortesao, R.: Physical feasibility of robot
% base inertial parameter identification: A linear matrix inequality approach (2014)

% %VERSIONINFO%

function [K, K_d, P_b, P_d] = %FN%(pkin)

