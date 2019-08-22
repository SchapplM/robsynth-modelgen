% Regressormatrix der Massenmatrix der PKM-Plattform (für 6FG)
% % Bezogen auf Inertialparameter
% 
% Eingabe:
% phi [3x1]
%   XYZ-Euler-Winkel des Plattform-KS
% 
% Ausgabe:
% M [6*6 x 10]
%   Regressor der Massenmatrix (Bezogen auf Winkelgeschwindigkeit im
%   Basis-KS und Moment in mitgedrehten Euler-Winkeln)
%   Massenmatrix Zeilenweise als Vektor definiert. Die entstehende
%   Massenmatrix ist nicht symmetrisch.
%   Reihenfolge der Inertialparameter: XX, XY, XZ, YY, YZ, ZZ, MX, MY, MZ, M

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function M_reg = rigidbody_pkm_pf_inertia_reg2(phi)

%% Initialisierung
x_all = [zeros(3,1); phi];

%% Berechnung
% Symbolische Berechnung in robot_para_plattform_rotmat_dynamics_regressor.mw
% Aus invdyn_floatb_twist_Mplatform_matlab.m
t194 = x_all(5);
t191 = cos(t194);
t188 = sin(t194);
t193 = x_all(6);
t190 = cos(t193);
t195 = x_all(4);
t192 = cos(t195);
t209 = t192 * t190;
t187 = sin(t193);
t189 = sin(t195);
t217 = t189 * t187;
t171 = t188 * t209 - t217;
t219 = t171 * t187;
t210 = t192 * t187;
t216 = t189 * t190;
t170 = t188 * t210 + t216;
t220 = t170 * t190;
t158 = (-t219 + t220) * t191;
t214 = t190 * t188;
t169 = t189 * t214 + t210;
t228 = (t169 * t192 - t171 * t189) * t191;
t224 = t169 * t187;
t167 = t188 * t217 - t209;
t225 = t167 * t190;
t157 = (-t224 + t225) * t191;
t227 = t167 * t187;
t226 = t167 * t188;
t223 = t169 * t188;
t222 = t170 * t187;
t221 = t170 * t188;
t218 = t188 * t171;
t215 = t189 * t191;
t183 = t187 ^ 2;
t213 = t191 * t183;
t212 = t191 * t187;
t211 = t191 * t190;
t208 = t192 * t191;
t186 = t191 ^ 2;
t207 = t186 * t216;
t206 = t189 * t212;
t205 = t189 * t211;
t204 = t170 * t212;
t203 = t187 * t211;
t202 = t186 * t210;
t201 = t190 * t208;
t175 = t186 * t209;
t200 = t175 + t218;
t199 = t171 * t167 - t170 * t169;
t160 = t207 + t223;
t185 = t190 ^ 2;
t184 = t188 ^ 2;
t181 = t191 * t185;
t180 = t186 * t185;
t179 = t186 * t183;
t177 = t187 * t188;
t176 = t187 * t208;
t174 = t186 * t217;
t173 = t188 * t208;
t166 = t171 * t190;
t165 = t169 * t190;
t164 = t171 * t211;
t163 = t169 * t211;
t161 = t202 + t221;
t159 = -t174 - t226;
t156 = (-t167 * t192 + t170 * t189) * t191;
t155 = -t164 + t173 - t204;
t154 = t163 + (-t188 * t189 + t227) * t191;
t153 = -t189 * t186 * t192 - t167 * t170 - t169 * t171;
t1 = [0, 0, 0, 0, 0, 0, 0, 0, 0, t180 + t179 + t184; 0, 0, 0, 0, 0, 0, 0, 0, 0, t154; 0, 0, 0, 0, 0, 0, 0, 0, 0, t155; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, -t214, t177, t181 + t213, 0; 0, 0, 0, 0, 0, 0, -t212, -t211, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, t154; 0, 0, 0, 0, 0, 0, 0, 0, 0, t189 ^ 2 * t186 + t167 ^ 2 + t169 ^ 2; 0, 0, 0, 0, 0, 0, 0, 0, 0, t153; 0, 0, 0, 0, 0, 0, t159, -t160, t157, 0; 0, 0, 0, 0, 0, 0, t205, -t206, t165 + t227, 0; 0, 0, 0, 0, 0, 0, -t167, -t169, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, t155; 0, 0, 0, 0, 0, 0, 0, 0, 0, t153; 0, 0, 0, 0, 0, 0, 0, 0, 0, t192 ^ 2 * t186 + t170 ^ 2 + t171 ^ 2; 0, 0, 0, 0, 0, 0, t161, t200, -t158, 0; 0, 0, 0, 0, 0, 0, -t201, t176, -t166 - t222, 0; 0, 0, 0, 0, 0, 0, t170, t171, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, t159, -t160, t157, 0; 0, 0, 0, 0, 0, 0, t161, t200, -t158, 0; t180, -0.2e1 * t186 * t187 * t190, 0.2e1 * t188 * t211, t179, -0.2e1 * t188 * t212, t184, 0, 0, 0, 0; t203, t181 - t213, t177, -t203, t214, 0, 0, 0, 0, 0; 0, 0, t211, 0, -t212, t188, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, -t159, t160, -t157, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, -t156, t228, t199, 0; t163, (-t224 - t225) * t191, -t207 + t223, t167 * t212, t174 - t226, -t188 * t215, 0, 0, 0, 0; t224, t165 - t227, -t206, -t225, -t205, 0, 0, 0, 0, 0; 0, 0, t169, 0, -t167, -t215, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, -t161, -t200, t158, 0; 0, 0, 0, 0, 0, 0, t156, -t228, -t199, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; -t164, (t219 + t220) * t191, t175 - t218, -t204, -t202 + t221, t173, 0, 0, 0, 0; -t219, -t166 + t222, t176, t220, t201, 0, 0, 0, 0, 0; 0, 0, -t171, 0, t170, t208, 0, 0, 0, 0;];
M_reg = t1;