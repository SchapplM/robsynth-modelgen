
# Lagrange Formalis for Robot based on MDH frames
# Einleitung
# Berechnung des Lagrange-Formalismus
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# tree -> Berechnung für eine beliebige Baumstruktur (ohne Schleifen)
# floatb -> floating base wird durch base twist (Geschwindigkeit der Basis) oder vollständige Orientierung (Euler-Winkel) berücksichtigt
# rotmat -> Kinematik wird mit Rotationsmatrizen berechnet
# lagrange -> Berechnung des Lagrange-Formalismus
# worldframe -> Berechnung basierend auf Energien aus Welt-KS (KS W)
# par12 -> Parametersatz 1 (Schwerpunkt als Parameter: SX,SY,SZ) oder Parametersatz 2 (1. und 2. Moment MX,MY,MZ,...)
# Autor
# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# Sources
# [GautierKhalil1990] Direct Calculation of Minimum Set of Inertial Parameters of Serial Robots
# [KhalilDombre2002] Modeling, Identification and Control of Robots
# [Ortmaier2014] Vorlesungsskript Robotik I
# Initialization
interface(warnlevel=0): # Unterdrücke die folgende Warnung.
restart: # Gibt eine Warnung, wenn über Terminal-Maple mit read gestartet wird.
interface(warnlevel=3):
with(LinearAlgebra):
with(ArrayTools):
with(codegen):
with(CodeGeneration):
with(StringTools):
# Einstellungen für Code-Export: Optimierungsgrad (2=höchster) und Aktivierung jedes Terms.
codegen_opt := 2:
read "../helper/proc_convert_s_t":
read "../helper/proc_convert_t_s": 
read "../helper/proc_MatlabExport":
read "../helper/proc_Lagrange1":
read "../transformation/proc_rotx": 
read "../transformation/proc_roty": 
read "../transformation/proc_rotz": 
read "../transformation/proc_trotx": 
read "../transformation/proc_troty": 
read "../transformation/proc_trotz": 
read "../transformation/proc_transl": 
read "../transformation/proc_trafo_mdh": 
read "../robot_codegen_definitions/robot_env":
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name):
# Kennung des Parametersatzes, für den die Dynamikfunktionen erstellt werden sollen. Muss im Repo und in der mpl-Datei auf 1 gelassen werden, da die folgende Zeile mit einem Skript verarbeitet wird.
codegen_dynpar := 1:
# Ergebnisse der Energie laden
if codegen_dynpar = 1 then
  read sprintf("../codeexport/%s/tmp/energy_potential_floatb_%s_worldframe_par1_maple.m", robot_name, base_method_name):
  read sprintf("../codeexport/%s/tmp/energy_kinetic_floatb_%s_worldframe_par1_maple.m", robot_name, base_method_name):
elif codegen_dynpar = 2 then
  read sprintf("../codeexport/%s/tmp/energy_potential_floatb_%s_worldframe_par2_maple.m", robot_name, base_method_name):
  read sprintf("../codeexport/%s/tmp/energy_kinetic_floatb_%s_linkframe_par2_maple.m", robot_name, base_method_name):
else
  printf("Energiefunktionen nur für Parametersatz 1 oder 2 definiert\n"):
end:
T := T:
U_grav := U_grav:
printf("Generiere Terme des Lagrange-Formalismus für %s mit Parametersatz %d und %s\n", robot_name, codegen_dynpar, base_method_name):
# Lagrange Formalismus (mit Funktion)
OutputLagrange := Lagrange1(T, U_grav, NQ):
dTdqDdt_s := OutputLagrange[1]:
dTdq_s := OutputLagrange[2]:
dUdq_s := OutputLagrange[3]:
save dUdq_s, sprintf("../codeexport/%s/tmp/floatb_%s_lagrange_dUdq_s_par%d_maple.m", robot_name, base_method_name, codegen_dynpar):
save dTdq_s, sprintf("../codeexport/%s/tmp/floatb_%s_lagrange_dTdq_s_par%d_maple.m", robot_name, base_method_name, codegen_dynpar):
save dTdqDdt_s, sprintf("../codeexport/%s/tmp/floatb_%s_lagrange_dTdqDdt_s_par%d_maple.m", robot_name, base_method_name, codegen_dynpar):
printf("Lagrange-Formalismus durchgeführt und Ergebnisse gespeichert\n"):

