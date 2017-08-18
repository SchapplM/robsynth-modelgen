% Calculate minimal parameter regressor of potential energy for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% %INPUT_PKIN%
% 
% Output:
% U_reg [1x%NMPVFIXB%]
%   minimal parameter regressor of Potential energy

% %VERSIONINFO%

function U_reg = %FN%(qJ, g, ...
  pkin)
