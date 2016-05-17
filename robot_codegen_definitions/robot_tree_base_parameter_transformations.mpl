# Base Parameter Properties for Robot based on MDH frames
# Einleitung
# Untersuchung von Eigenschaften der Regressorform und des Minimalparametervektors
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# tree -> Berechnung für eine allgemeine Baumstruktur
# base_parameter_transformations -> Umwandlung der Minimalparameterform in andere Darstellungen
# Authors
# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-04
# 
# Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# Quellen
# [SousaCor2014] Sousa, C. D. and Cortesao, R.: Physical feasibility of robot base inertial parameter identification: A linear matrix inequality approach (2014)
# Initialisierung
restart:
with(LinearAlgebra):
with(ArrayTools):
with(codegen):
with(CodeGeneration):
with(StringTools):
codegen_opt := true:
codegen_act := true:
read "../helper/proc_convert_s_t":
read "../helper/proc_convert_t_s": 
read "../helper/proc_MatlabExport":
read "../robot_codegen_definitions/robot_env":
printf("Generiere Minimalparameterregressor der Energie für %s\n", robot_name, codegen_dynpar):
read sprintf("../codeexport/%s_tree_floatb_twist_definitions", robot_name):
# Ergebnisse der Minimalparametergruppierung laden
read sprintf("../codeexport/%s_minimal_parameter_vector_maple", robot_name):
Paramvec2 := Paramvec2:
# Minimalparametervektor als Matrixdarstellung
# Siehe atlas_limb_sym_codegen_fixedbase_mdh_dynamics_regressor_minpar.mw
# Siehe [SousaCor2014] equ. (38)
# Beispielrechnung:
# U=dU/dPV2*PV2=dU/dMPV*MPV # Parameterlineare Form
# MPV=dMPV/dPV2 * PV2 # Minimalparametervektor MPV hängt nur linear von den normalen Parametern im Parametervektor PV2 ab
# dU/dMPV = dU/dPV2 * dMPV/dPV1
Paramvec_size := RowDimension(Paramvec2):
dMPVdPV2 := Matrix(Paramvec_size, 10*NL):
for i to Paramvec_size do 
  for j to 10*NL do 
    dMPVdPV2[i, j] := diff(Paramvec2[i, 1], PV2_vec[j, 1]):
  end do:
end do:
# Export der Umwandlung von Parametersatz 2 nach Minimalparameter (Matrix)
MatlabExport(dMPVdPV2, sprintf("../codeexport/%s_minparvec_diff_wrt_par2_matlab.m", robot_name), true):
save dMPVdPV2, sprintf("../codeexport/%s_minparvec_diff_wrt_par2_maple", robot_name):

