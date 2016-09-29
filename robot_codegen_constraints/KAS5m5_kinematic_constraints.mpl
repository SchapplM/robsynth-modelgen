# Kinematik-Berechnung 3. Arm KAS5
# Beschreibung
# 
# Modell m5: 
# * Kurbel N7 ist starr mit Körper K2 gekoppelt (über die Achse) (wie m3), 
# * Zahnradkopplung zwischen Körpern Z2 und Z3 (Unterschied zu m3)
# 
# Setze die Zwangsbedingung (Zahnradkopplung) in die bereits berechneten Zwangsbedingungen für das KAS5_m3 ein
# 
# Berechnung der Kopplung der Zahnräder am Ellenbogen
# Autor
# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-02
# Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# Initialisierung
restart:
kin_constraints_exist := true: # Für Speicherung
with(LinearAlgebra):
with(StringTools):
read "../helper/proc_convert_s_t":
read "../helper/proc_convert_t_s": 
read "../helper/proc_MatlabExport":
read "../robot_codegen_constraints/proc_subs_kintmp_exp":
read "../robot_codegen_definitions/robot_env":
read sprintf("../codeexport/%s_tree_floatb_definitions", robot_name):
# Lese Ergebnisse der Kinematik von KAS5_m3 aus.
# Siehe KAS5_m3_sym_codegen_kinematic_constraints.mw
# kintmp_qs, kintmp_qt, lpar_qs, lpar_qt, kintmp_subsexp
read  sprintf("../codeexport/KAS5m3_kinematic_constraints_maple_inert.m", robot_name):
kintmp_qs_KAS5m3 := kintmp_qs:
kintmp_qt_KAS5m3 := kintmp_qt:
lpar_qs_KAS5m3 := lpar_qs:
lpar_qt_KAS5m3 := lpar_qt:
kintmp_subsexp_KAS5m3 := kintmp_subsexp:
# Wende Zwangsbedingungen aus Zahnradkontakt an
# Wandle die verallgemeinerten Koordinaten um. Benutze die Reihenfolge für KAS5_m5.
# Funktion zur Ersetzung der Gelenkwinkel (allgemein -> m3)
subs_q_J_m3 := proc(A)
  local A_s, j; 
  A_s := A; 
  for j from 1 to RowDimension(qJ_s) do
    A_s := subs({qJ_s(j,1) = qJs_KAS5m3(j,1)}, A_s):
  end do:
  return A_s:
end proc:
qJs_KAS5m3:=Matrix(7, 1, [qJs1_m3, qJs2_m3, qJs3_m3, qJs4_m3, qJs5_m3, qJs6_m3, qJs7_m3]):
kintmp_subsexp_KAS5m3 := subs_q_J_m3(kintmp_subsexp_KAS5m3):
kintmp_qs_KAS5m3 := subs_q_J_m3(kintmp_qs_KAS5m3):
lpar_qs_KAS5m3 := subs_q_J_m3(lpar_qs_KAS5m3):
# Funktion zur Ersetzung der verallgemeinerten Koordinaten (m3 -> m5)
subs_q_m3_m5 := proc(A)
  local A_s, j; 
  A_s := A; 
  for j from 1 to RowDimension(qJ_s) do
    A_s := subs({qJs_KAS5m3(j,1) = qJs_KAS5m3m5subs(j,1)}, A_s):
  end do:
  return A_s:
end proc:
qJs_KAS5m3m5subs:=Matrix(7, 1, [qJs1_m5, qJs2_m5, qJs3_m5, qJs4_m5, qJs4_m5-qJs3_m5+delta18s, qJs5_m5, 0]):
kintmp_subsexp_KAS5m5 := subs_q_m3_m5(kintmp_subsexp_KAS5m3):
kintmp_qs_KAS5m5 := subs_q_m3_m5(kintmp_qs_KAS5m3):
lpar_qs_KAS5m5 := subs_q_m3_m5(lpar_qs_KAS5m3):
# Konvertiere Ergebnisse
# Rückersetzung der markierten Koordinaten gegen die normale Form (qJ_KAS5m5 -> qJ)
subs_q_m5_J := proc(A)
  local A_s, j; 
  A_s := A; 
  for j from 1 to RowDimension(qJs_KAS5m5) do
    A_s := subs({qJs_KAS5m5(j,1) = qJ_s(j,1)}, A_s):
  end do:
  return A_s:
end proc:
qJs_KAS5m5:=Matrix(5, 1, [qJs1_m5, qJs2_m5, qJs3_m5, qJs4_m5, qJs5_m5]):
kintmp_subsexp := subs_q_m5_J(kintmp_subsexp_KAS5m5):
kintmp_qs := subs_q_m5_J(kintmp_qs_KAS5m5):
lpar_qs := subs_q_m5_J(lpar_qs_KAS5m5):
kintmp_qt := convert_s_t(kintmp_qs):
lpar_qt := convert_s_t(lpar_qs):
# Speichern der Ergebnisse
save kin_constraints_exist, kintmp_qs, kintmp_qt, lpar_qs, lpar_qt, kintmp_subsexp, sprintf("../codeexport/%s_kinematic_constraints_maple_inert", robot_name):
save kin_constraints_exist, kintmp_qs, kintmp_qt, lpar_qs, lpar_qt, kintmp_subsexp, sprintf("../codeexport/%s_kinematic_constraints_maple_inert.m", robot_name):
# Liste der Parameter speichern
# Liste mit abhängigen konstanten Kinematikparametern erstellen (wichtig für Matlab-Funktionsgenerierung)
read "../helper/proc_list_constant_expressions";
kc_symbols := Matrix(list_constant_expressions( kintmp_subsexp(..,2) )):
save kc_symbols, sprintf("../codeexport/%s_kinematic_constraints_symbols_list_maple", robot_name):
MatlabExport(Transpose(kc_symbols), sprintf("../codeexport/%s_kinematic_constraints_symbols_list_matlab.m", robot_name), 2):
printf("Zwangsbedingungen der Parallelstruktur von KAS5_m3 nach KAS5_m5 angepasst. %s\n", FormatTime("%Y-%m-%d %H:%M:%S")):

