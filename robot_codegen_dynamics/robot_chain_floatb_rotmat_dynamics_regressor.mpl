# Base Parameter Regresor Inverse Dynamics for Robot based on MDH frames
# Einleitung
# Berechnung der inversen Dynamik in Regressorform
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# chain -> Berechnung für eine serielle Struktur (nicht: Baumstruktur)
# fixb -> fixed base. Kein Floating base Modell. Dort ist diese Form der Minimalparameterform nicht möglich.
# rotmat -> Kinematik wird mit Rotationsmatrizen berechnet
# dynamics -> Berechnung der Dynamik
# regressor -> Regressorform (parameterlinear)
# 
# Siehe auch: robot_chain_floatb_rotmat_dynamics_regressor.mw. Im Gegensatz dazu werden hier die fixed-Base-Ausdrücke ohne eine "twist"-Basis-Berechnung übernommen, die sowieso falsch ist.
# Autor
# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
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
codeexport_grav := true: 
codeexport_corvec := true:
codeexport_cormat := true:
codeexport_inertia := true:
codeexport_inertiaD := true:
codeexport_invdyn := true:
read "../helper/proc_convert_s_t":
read "../helper/proc_convert_t_s": 
read "../helper/proc_MatlabExport":
read "../helper/proc_LagrangeN":
read "../helper/proc_index_symmat2vector":
read "../transformation/proc_rotx": 
read "../transformation/proc_roty": 
read "../transformation/proc_rotz": 
read "../transformation/proc_trotx": 
read "../transformation/proc_troty": 
read "../transformation/proc_trotz": 
read "../transformation/proc_transl": 
read "../transformation/proc_trafo_mdh": 
read "../robot_codegen_definitions/robot_env":
read sprintf("../codeexport/%s/tree_floatb_definitions", robot_name):
# Mit diesem Arbeitsblatt werden die Regressor-Ausdrücke für Fixed-Base und Floating-Base Modelle generiert. Erkenne welche Basis-Modellierung aktiv ist
if base_method_name="twist" then # Basis-Methode "twist" wird (hier) nur für fixed Base benutzt
  expstring:="fixb":
elif base_method_name="eulangrpy" then 
  expstring:="floatb_eulangrpy":
else
  printf("Nicht behandelte Basis-Methode: %s\n", base_method_name):
fi:
# Es ist in diesem Arbeitsblatt möglich, zwei verschiedene Regressoren zu generieren und zu exportieren: Basierend auf Minimalparametern und auf vollem Parametersatz (PV2).
# Der Term "regressor" oder "regressor_minpar" ist jeweils in den Dateinamen enthalten
regressor_modus := "regressor":
if regressor_modus = "regressor_minpar" then
  read sprintf("../codeexport/%s/energy_kinetic_%s_regressor_minpar_maple.m", robot_name, expstring):
  read sprintf("../codeexport/%s/energy_potential_%s_regressor_minpar_maple.m", robot_name, expstring):
  t_ges := t_ges_minpar:
  u_ges := u_ges_minpar:
  printf("Generiere Minimalparameterregressor der Dynamik für %s\n", robot_name):
end if:
if regressor_modus = "regressor" then
  read sprintf("../codeexport/%s/energy_kinetic_%s_regressor_maple.m", robot_name, expstring):
  read sprintf("../codeexport/%s/energy_potential_%s_regressor_maple.m", robot_name, expstring):
  t_ges := t_ges:
  u_ges := u_ges:
  printf("Generiere Regressor der Dynamik für %s (nicht Minimalparameter)\n", robot_name):
end if:
# Schalter zur Auswahl der unterschiedlichen Terme, die exportiert werden sollen. Für parallele Berechnung interessant.
DynString := "Term:":
if codeexport_grav then
  DynString := sprintf("%s g",DynString):
end if:
if codeexport_corvec then
  DynString := sprintf("%s c",DynString):
end if:
if codeexport_cormat then
  DynString := sprintf("%s C",DynString):
end if:
if codeexport_inertia then
  DynString := sprintf("%s M",DynString):
end if:
if codeexport_inertiaD then
  DynString := sprintf("%s MD",DynString):
end if:
if codeexport_invdyn then
  DynString := sprintf("%s tau",DynString):
end if:
printf("Generiere Dynamik-Regressor (%s) für %s und %s\n", DynString, robot_name, base_method_name):
# Lagrange Formalismus (mit Funktion)
OutputLagrange := LagrangeN(t_ges, u_ges):
dTdqDdt_s := OutputLagrange[1]:
dTdq_s := OutputLagrange[2]:
dUdq_s := OutputLagrange[3]:
save dUdq_s, sprintf("../codeexport/%s/floatb_lagrange_dUdq_s_%s_maple.m", robot_name, regressor_modus):
save dTdq_s, sprintf("../codeexport/%s/floatb_lagrange_dTdq_s_%s_maple.m", robot_name, regressor_modus):
save dTdqDdt_s, sprintf("../codeexport/%s/floatb_lagrange_dTdqDdt_s_%s_maple.m", robot_name, regressor_modus):
# Extraktion einzelner Terme
# Gravitational Load
# Generate
taug_regressor_s := dUdq_s:
save taug_regressor_s, sprintf("../codeexport/%s/gravload_%s_maple.m", robot_name, regressor_modus):
# Matlab Export
# Belastung der Basis (nur Floating Base)
if codeexport_grav and not(base_method_name="twist") then
  MatlabExport(taug_regressor_s(1..6,..), sprintf("../codeexport/%s/base_gravload_floatb_%s_%s_matlab.m", robot_name, base_method_name, regressor_modus), codegen_opt):
end if:
# Belastung der Gelenke: Fixed Base
if codeexport_grav and base_method_name="twist" then
  MatlabExport(taug_regressor_s, sprintf("../codeexport/%s/joint_gravload_fixb_%s_matlab.m", robot_name, regressor_modus), codegen_opt):
end if:
# Belastung der Gelenke: Floating Base (Eingang ist jetzt der Gravitationsvektor im Welt-KS, nicht mehr im Basis-KS)
if codeexport_grav and not(base_method_name="twist") then
  MatlabExport(taug_regressor_s(7..NQ,..), sprintf("../codeexport/%s/joint_gravload_floatb_%s_%s_matlab.m", robot_name, base_method_name, regressor_modus), codegen_opt):
end if:
# Kompletter Vektor
if codeexport_grav and not(base_method_name="twist") then
  MatlabExport(taug_regressor_s(1..NQ,..), sprintf("../codeexport/%s/gravload_floatb_%s_%s_matlab.m", robot_name, base_method_name, regressor_modus), codegen_opt):
end if:
# Mass Matrix
# Initialisiere. Speichere nur den unteren linken Teil der Massenmatrix
# Siehe: https://de.wikipedia.org/wiki/Symmetrische_Matrix
Paramvec_size := ColumnDimension(t_ges):
# Berechnung Gelenkmassenmatrix
tauMMj_regressor_s := dTdqDdt_s(7..NQ,..):
MMjj_regressor_s := Matrix(NQJ*(NQJ+1)/2, Paramvec_size):
i_rr := 0:
for i to NQJ do # Zeilenindex der Massenmatrix
  for j to NQJ do  # Spaltenindex der Massenmatrix
    if j > i then
      next: # rechte obere Seite der symmetrischen Matrix. Keine neue Information. Nicht berechnen oder speichern.
    end if:
    i_rr := i_rr + 1: # Gehe zeilenweise durch den unteren linken Teil der Massenmatrix (inkl. Diagonale)
    for k to Paramvec_size do # Spaltenindex der Regressormatrix
      MMjj_regressor_s[i_rr, k] := diff(tauMMj_regressor_s[i, k], qJDD_s[j, 1]):
    end do:
  end do:
end do:
# Export Gelenkmassenmatrix
if codeexport_inertia and base_method_name="twist" then
  MatlabExport(MMjj_regressor_s, sprintf("../codeexport/%s/inertia_joint_joint_fixb_%s_matlab.m", robot_name, regressor_modus), codegen_opt):
end if:
# Berechnung vollständige Massenmatrix
tauMM_regressor_s := dTdqDdt_s(1..NQ,..):
MM_regressor_s := Matrix(NQ*(NQ+1)/2, Paramvec_size):
i_rr := 0:
for i to NQ do # Zeilenindex der Massenmatrix
  for j to NQ do  # Spaltenindex der Massenmatrix
    if j > i then
      next: # rechte obere Seite der symmetrischen Matrix. Keine neue Information. Nicht berechnen oder speichern.
    end if:
    i_rr := i_rr + 1: # Gehe zeilenweise durch den unteren linken Teil der Massenmatrix (inkl. Diagonale)
    for k to Paramvec_size do # Spaltenindex der Regressormatrix
      MM_regressor_s[i_rr, k] := diff(tauMM_regressor_s[i, k], qDD_s[j, 1]):
    end do:
  end do:
end do:
# Export Gesamt-Massenmatrix
if codeexport_inertia and not(base_method_name="twist") then
  MatlabExport(MM_regressor_s, sprintf("../codeexport/%s/inertia_floatb_%s_%s_matlab.m", robot_name, base_method_name, regressor_modus), codegen_opt):
end if:
# Gelenk-Basis-Massenmatrix (untere linke Teilmatrix der Gesamt-Massenmatrix
MMjb_regressor_s := Matrix(6*NQJ, Paramvec_size):
# Gehe Schleife über alle Massenmatrix-Elemente durch und entnehme die passenden Elemente für die Teilmatrix)
itmp:=0:
for i to NQ do
  for j to NQ do  # Spaltenindex der Massenmatrix
    if i > 6 and j < 7 then # unterer linker Teil
      i_MM := index_symmat2vec(NQ,i,j): # Passender Index für zeilenweise ausgewählten symmetrischen Teil
      itmp := itmp + 1:
      MMjb_regressor_s[itmp,..] := MM_regressor_s[i_MM,..]:
    end if:
  end do:
end do:
if codeexport_inertia and not(base_method_name="twist") then
  MatlabExport(MMjb_regressor_s, sprintf("../codeexport/%s/inertia_joint_base_floatb_%s_%s_matlab.m", robot_name, base_method_name, regressor_modus), codegen_opt):
end if:
# Mass Matrix Time Derivative
# Konvertiere Massenmatrix in zeitabhängige Variablen, um Zeitableitung zu berechnen
MMjj_regressor_t := convert_s_t(MMjj_regressor_s):
MMDjj_regressor_t := diff~(MMjj_regressor_t, t):
MMDjj_regressor_s := convert_t_s(MMDjj_regressor_t):
# Matlab Export
if codeexport_inertia and base_method_name="twist" then
  MatlabExport(MMDjj_regressor_s, sprintf("../codeexport/%s/inertiaD_joint_joint_%s_matlab.m", robot_name, regressor_modus), codegen_opt):
end if:
# Coriolis Vector
# Generate
tauC_regressor_s := dTdqDdt_s-dTdq_s:
for i to NQ do 
  tauC_regressor_s := subs({qDD_s(i, 1) = 0}, tauC_regressor_s):
end do:
save tauC_regressor_s, sprintf("../codeexport/%s/coriolisvec_joint_fixb_%s_maple.m", robot_name, regressor_modus):
# Matlab Export
# Belastung der Gelenke
if codeexport_corvec and base_method_name="twist" then
  MatlabExport(tauC_regressor_s(7..NQ,..), sprintf("../codeexport/%s/coriolisvec_joint_fixb_%s_matlab.m", robot_name, regressor_modus), codegen_opt):
end if:
# Coriolis Matrix
# Calculation with Christoffel Symbol approach
# [KhalilDombre2002], equ. (9.7) (p. 195)
# Funktion zur Anwendung des Algorithmus aus [KhalilDombre2002]:
cijk := proc (i::integer, j::integer, k::integer, A, qs)
  local c:
  c := (1/2)*(diff(A[i, j], qs(k, 1)))+(1/2)*(diff(A[i, k], qs(j, 1)))-(1/2)*(diff(A[j, k], qs(i, 1))):
  return c:
end proc:
# Initialisierung. Speichere die vollständige Coriolismatrix (nicht symmetrisch/schiefsymmetrisch). Unterschied zum Vorgehen bei der Massenmatrix
Cjj_regressor_s := Matrix(NQJ*NQJ, Paramvec_size):
# Berechnung
i_rr := 0: # Vektor-Index für den Regressor der Coriolismatrix (Ausgabe)
for i to NQJ do # Zeilenindex der Coriolismatrix
  for j to NQJ do  # Spaltenindex der Coriolismatrix
    i_rr := i_rr + 1: # Gehe zeilenweise durch den unteren linken Teil der Coriolismatrix (inkl. Diagonale)
    for k from 1 to Paramvec_size do # Spaltenindex der Regressormatrix
      # Massenmatrix für Parameter k generieren (für Funktion mit Christoffel-Symbol-Ansatz benötigt)
      MM_jj_k := Matrix(NQJ,NQJ):
      for ii from 1 to NQJ do
        for jj from 1 to NQJ do
          MM_jj_k(ii,jj) := MMjj_regressor_s(index_symmat2vec(NQJ, ii, jj), k):
        end do:
      end do:
      for l from 1 to NQJ do # Siehe [KhalilDombre2002]
        Cjj_regressor_s[i_rr, k] := Cjj_regressor_s[i_rr,k] + cijk(i,j,l,MM_jj_k,qJ_s)*qJD_s[l,1]:
      end do:
    end do:
  end do:
end do:
save Cjj_regressor_s, sprintf("../codeexport/%s/coriolismat_joint_fixb_%s_maple.m", robot_name, regressor_modus):
# Matlab Export: Fixed base
if codegen_act then
  MatlabExport(Cjj_regressor_s, sprintf("../codeexport/%s/coriolismat_joint_fixb_%s_matlab.m", robot_name, regressor_modus), codegen_opt):
end if:
# Inverse Dynamics
# Generate
tau_regressor_s := dTdqDdt_s-dTdq_s+dUdq_s:
save tau_regressor_s, sprintf("../codeexport/%s/invdyn_joint_fixb_%s_maple.m", robot_name, regressor_modus):
# Matlab Export
# Belastung der Gelenke
if codegen_act then
  MatlabExport(tau_regressor_s(7..NQ,..), sprintf("../codeexport/%s/invdyn_joint_fixb_%s_matlab.m", robot_name, regressor_modus), codegen_opt):
end if:

