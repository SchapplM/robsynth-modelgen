# Kinematik-Berechnung 3. Arm KAS5
# Beschreibung
# 
# Modell m6: 
# * Kurbel N7 ist drehbar gegen Körper K2 gelagert (über die Achse) (wie m4), 
# * Zahnradkopplung zwischen Körpern Z2 und Z3 (Unterschied zu m4)
# 
# Setze die Zwangsbedingung (Zahnradkopplung) in die bereits berechneten Zwangsbedingungen für das KAS5_m4 ein
# 
# Berechnung der Kopplung der Zahnräder am Ellenbogen
# 
# Vorlage für dieses Skript:
# KAS5_m5_sym_codegen_kinematic_constraints.mw
# Autor
# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# Initialisierung
restart:
kin_constraints_exist := true: # Für Speicherung
with(LinearAlgebra):
with(StringTools):
codegen_act := true:
codegen_opt := 1:
read "../helper/proc_MatlabExport":
read "../helper/proc_convert_s_t":
read "../helper/proc_convert_t_s":
read "../robot_codegen_constraints/proc_subs_kintmp_exp":
with(RealDomain):
read "../robot_codegen_definitions/robot_env":
read sprintf("../codeexport/%s_tree_floatb_definitions", robot_name):
# 
# Lese Ergebnisse der Kinematik von KAS5_m4 aus.
# Siehe KAS5_m4_sym_codegen_kinematic_constraints.mw
# kintmp_qs, kintmp_qt, lpar_qs, lpar_qt, kintmp_subsexp
read  sprintf("../codeexport/KAS5m4_kinematic_constraints_maple_inert.m", robot_name):
kintmp_qs_KAS5m4 := kintmp_qs:
kintmp_qt_KAS5m4 := kintmp_qt:
lpar_qs_KAS5m4 := lpar_qs:
lpar_qt_KAS5m4 := lpar_qt:
kintmp_subsexp_KAS5m4 := kintmp_subsexp:
# Wende Zwangsbedingungen aus Zahnradkontakt an
# Wandle die verallgemeinerten Koordinaten um. Benutze die Reihenfolge für KAS5_m6.
# Funktion zur Ersetzung der Gelenkwinkel (allgemein -> m4)
subs_q_J_m4 := proc(A)
  local A_s, j; 
  A_s := A; 
  for j from 1 to RowDimension(qJ_s) do
    A_s := subs({qJ_s(j,1) = qJs_KAS5m4(j,1)}, A_s):
  end do:
  return A_s:
end proc:
qJs_KAS5m4:=Matrix(7, 1, [qJs1_m4, qJs2_m4, qJs3_m4, qJs4_m4, qJs5_m4, qJs6_m4, qJs7_m4]):
kintmp_subsexp_KAS5m4 := subs_q_J_m4(kintmp_subsexp_KAS5m4):
kintmp_qs_KAS5m4 := subs_q_J_m4(kintmp_qs_KAS5m4):
lpar_qs_KAS5m4 := subs_q_J_m4(lpar_qs_KAS5m4):
# Funktion zur Ersetzung der verallgemeinerten Koordinaten (m4 -> m6)
subs_q_m4_m6 := proc(A)
  local A_s, j; 
  A_s := A; 
  for j from 1 to RowDimension(qJ_s) do
    A_s := subs({qJs_KAS5m4(j,1) = qJs_KAS5m4m6subs(j,1)}, A_s):
  end do:
  return A_s:
end proc:
qJs_KAS5m4m6subs:=Matrix(7, 1, [qJs1_m6, qJs2_m6, qJs3_m6, qJs4_m6, qJs5_m6, qJs5_m6+delta18s-qJs4_m6, qJs6_m6]):
kintmp_subsexp_KAS5m6 := subs_q_m4_m6(kintmp_subsexp_KAS5m4):
kintmp_qs_KAS5m6 := subs_q_m4_m6(kintmp_qs_KAS5m4):
lpar_qs_KAS5m6 := subs_q_m4_m6(lpar_qs_KAS5m4):
# Konvertiere Ergebnisse
# Rückersetzung der markierten Koordinaten gegen die normale Form (qJ_KAS5m6 -> qJ)
subs_q_m6_J := proc(A)
  local A_s, j; 
  A_s := A; 
  for j from 1 to RowDimension(qJs_KAS5m6) do
    A_s := subs({qJs_KAS5m6(j,1) = qJ_s(j,1)}, A_s):
  end do:
  return A_s:
end proc:
qJs_KAS5m6:=Matrix(5, 1, [qJs1_m6, qJs2_m6, qJs3_m6, qJs4_m6, qJs5_m6]):
kintmp_subsexp := subs_q_m6_J(kintmp_subsexp_KAS5m6):
kintmp_qs := subs_q_m6_J(kintmp_qs_KAS5m6):
lpar_qs := subs_q_m6_J(lpar_qs_KAS5m6):
kintmp_qt := convert_s_t(kintmp_qs):
lpar_qt := convert_s_t(lpar_qs):
# Speichern der Ergebnisse
save kin_constraints_exist, kintmp_qs, kintmp_qt, lpar_qs, lpar_qt, kintmp_subsexp, sprintf("../codeexport/%s_kinematic_constraints_maple_inert", robot_name):
save kin_constraints_exist, kintmp_qs, kintmp_qt, lpar_qs, lpar_qt, kintmp_subsexp, sprintf("../codeexport/%s_kinematic_constraints_maple_inert.m", robot_name):
printf("Zwangsbedingungen der Parallelstruktur von KAS5_m4 nach KAS5_m6 angepasst und gespeichert. %s\n", FormatTime("%Y-%m-%d %H:%M:%S")):
# Liste der Parameter speichern
# Liste mit abhängigen konstanten Kinematikparametern erstellen (wichtig für Matlab-Funktionsgenerierung)
read "../helper/proc_list_constant_expressions";
kc_symbols := Matrix(list_constant_expressions( kintmp_subsexp(..,2) )):
save kc_symbols, sprintf("../codeexport/%s_kinematic_constraints_symbols_list", robot_name):
# Exportieren der Ersetzungsausdrücke
# Zum Testen
# Exportieren des vollständigen Ausdruckes
if codegen_act then
  MatlabExport(kintmp_subsexp(..,2), sprintf("../codeexport/KAS5_m6_kinematik_parallel_kintmp_subsexp_matlab.m"), codegen_opt):
end if:

