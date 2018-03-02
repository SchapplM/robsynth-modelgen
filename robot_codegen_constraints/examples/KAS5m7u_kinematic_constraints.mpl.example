
# Kinematik-Berechnung 3. Arm KAS5
# Beschreibung
# 
# Modell ohne Zwangsbedingungen. Diese Definitionsdatei erzeugt Dummy-Variablen, damit die Code-Generierung funktioniert.
# 
# Die Gelenkvariablen der Schnittgelenke sollen nicht berechnet werden und sind als konstante Ausdrücke in kintmp_s vorgegeben und werden später mit Null ersetzt.
# 

# Autor
# Moritz Schappler, schappler@imes.uni-hannover.de, 2018-02
# Institut fuer mechatronische Systeme, Leibniz Universitaet Hannover
# Initialisierung
restart:
kin_constraints_exist := true: # Für Speicherung
;
with(LinearAlgebra):
with(StringTools): # Für Zeitausgabe
;
read "../helper/proc_convert_s_t":
read "../helper/proc_convert_t_s": 
read "../helper/proc_MatlabExport":
codegen_act := true:
read "../robot_codegen_constraints/proc_subs_kintmp_exp":
read "../robot_codegen_definitions/robot_env":
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name):

kintmp_subsexp := Matrix(2*RowDimension(kintmp_s),2):
kin_constraints_exist := true:
# Dummy-Einträge für Schnitt-Gelenke
# Die Winkel der Schnittgelenke sind egal, daher keine Berechnung.
# Ersetze alle mit Null
kintmp_qs := kintmp_s*0:
kintmp_qt := kintmp_t*0:
# Speichern der Ergebnisse
save kin_constraints_exist, kintmp_qs, kintmp_qt, kintmp_subsexp, sprintf("../codeexport/%s/tmp/kinematic_constraints_maple_inert", robot_name):
save kin_constraints_exist, kintmp_qs, kintmp_qt, kintmp_subsexp, sprintf("../codeexport/%s/tmp/kinematic_constraints_maple_inert.m", robot_name):
# Liste der Parameter speichern
# Liste mit abhängigen konstanten Kinematikparametern erstellen (wichtig für Matlab-Funktionsgenerierung)
read "../helper/proc_list_constant_expressions";
kc_symbols := Matrix(list_constant_expressions( kintmp_subsexp(..,2) )):
save kc_symbols, sprintf("../codeexport/%s/tmp/kinematic_constraints_symbols_list_maple", robot_name):
kintmp_s:
