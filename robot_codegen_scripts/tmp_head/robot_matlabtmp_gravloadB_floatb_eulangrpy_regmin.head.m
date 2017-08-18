% Calculate base parameter regressor of Gravitation load on the base for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_PHIB%
% g [3x1]
%   gravitation vector in world frame [m/s^2]
% %INPUT_PKIN%
% 
% Output:
% taug_reg [6x(%NL%*10)]
%   base parameter regressor of base forces required to compensate gravitation load
%   base moment as Euler angle moment

% %VERSIONINFO%

function taug_reg = %FN%(qJ, phi_base, g, pkin)
