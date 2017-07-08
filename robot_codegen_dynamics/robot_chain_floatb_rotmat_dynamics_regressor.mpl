# Base Parameter Regresor Inverse Dynamics for Robot based on MDH frames
# Einleitung
# Berechnung der inversen Dynamik in Regressorform
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# chain -> Berechnung für eine serielle Struktur (nicht: Baumstruktur)
# floatb -> Floating Base (und auch fixed Base) Modell der Basis.
# rotmat -> Kinematik wird mit Rotationsmatrizen berechnet
# dynamics -> Berechnung der Dynamik
# regressor -> Regressorform (parameterlinear)
# 
# Siehe auch: robot_chain_floatb_rotmat_dynamics_regressor.mw.
# Im Gegensatz zu der allgemeinen Dynamik werden hier die fixed-Base-Ausdrücke ohne eine "twist"-Basis-Berechnung übernommen.
# Ursache ist, dass der Parametervektor und damit der Regressor sich zwischen Fixed-Base und Floating-Base unterscheidet und daher ein Ableiten des Fixed-Base falles aus der Floating-Base-Formulierung mit einfacher Basis-Darstellung ("twist") nicht möglich ist.
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
# Der Term "regressor" oder "regressor_minpar" ist jeweils in den Dateinamen enthalten.
# Der folgende Befehl muss immer auf "regressor_minpar" gesetzt sein, da diese Zeile durch das Skript robot_codegen_maple_preparation.sh ausgewertet und modifiziert wird.
regressor_modus := "regressor_minpar":
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
# Belastung der Gelenke: 
# Fixed-Base: Gravitationsvektor im Basis-KS
# Floating Base: Gravitationsvektor im Welt-KS, Basis-Orientierung berücksichtigt
if codeexport_grav then
  MatlabExport(taug_regressor_s(7..NQ,..), sprintf("../codeexport/%s/joint_gravload_%s_%s_matlab.m", robot_name, expstring, regressor_modus), codegen_opt):
end if:
# Kompletter Vektor
if codeexport_grav and not(base_method_name="twist") then
  MatlabExport(taug_regressor_s(1..NQ,..), sprintf("../codeexport/%s/gravload_floatb_%s_%s_matlab.m", robot_name, base_method_name, regressor_modus), codegen_opt):
end if:
# Mass Matrix
# Berechnung vollständige Massenmatrix
# Generiere die Massenmatrix erst für Floating-Base-Komplettsystem, dann entnehme die Teilmatrizen für Gelenk-Gelenk, Gelenk-Basis zum separaten Export
Paramvec_size := ColumnDimension(t_ges):
# Berechnung vollständige Massenmatrix
# Initialisiere. Speichere nur den unteren linken Teil der Massenmatrix
# Siehe: https://de.wikipedia.org/wiki/Symmetrische_Matrix
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
# Gelenk-Massenmatrix extrahieren
MMjj_regressor_s := Matrix(NQJ*(NQJ+1)/2, Paramvec_size):
itmp:=0:
for i to NQ do
  for j to NQ do  # Spaltenindex der Massenmatrix
    if j > i then
      next: # rechte obere Seite der symmetrischen Matrix. Keine neue Information. Nicht berechnen oder speichern.
    end if:
    if i > 6 and j > 6 then # unterer rechter Teil (Gelenkmoment-Gelenkbeschleunigung-Terme)
      i_MM := index_symmat2vec(NQ,i,j): # Passender Index für zeilenweise ausgewählten symmetrischen Teil (siehe Gesamt-Massenmatrix)
      itmp := itmp + 1:
      MMjj_regressor_s[itmp,..] := MM_regressor_s[i_MM,..]:
    end if:
  end do:
end do:
# Export Gelenkmassenmatrix
if codeexport_inertia and base_method_name="twist" then
  MatlabExport(MMjj_regressor_s, sprintf("../codeexport/%s/inertia_joint_joint_%s_%s_matlab.m", robot_name, expstring, regressor_modus), codegen_opt):
end if:
# Gelenk-Basis-Massenmatrix
#  (untere linke Teilmatrix der Gesamt-Massenmatrix)
MMjb_regressor_s := Matrix(6*NQJ, Paramvec_size):
# Gehe Schleife über alle Massenmatrix-Elemente durch und entnehme die passenden Elemente für die Teilmatrix)
itmp:=0:
for i to NQ do
  for j to NQ do  # Spaltenindex der Massenmatrix
    if i > 6 and j < 7 then # unterer linker Teil
      i_MM := index_symmat2vec(NQ,i,j): # Passender Index für zeilenweise ausgewählten symmetrischen Teil (siehe Gesamt-Massenmatrix)
      itmp := itmp + 1:
      MMjb_regressor_s[itmp,..] := MM_regressor_s[i_MM,..]:
    end if:
  end do:
end do:
if codeexport_inertia and not(base_method_name="twist") then
  MatlabExport(MMjb_regressor_s, sprintf("../codeexport/%s/inertia_joint_base_floatb_%s_%s_matlab.m", robot_name, base_method_name, regressor_modus), codegen_opt):
end if:
# Basis-Massenmatrix
MMbb_regressor_s := Matrix(6*(6+1)/2, Paramvec_size):
# Gehe Schleife über alle Massenmatrix-Elemente durch und entnehme die passenden Elemente für die Teilmatrix)
itmp:=0:
for i to NQ do
  for j to NQ do  # Spaltenindex der Massenmatrix
    if j > i then
      next: # rechte obere Seite der symmetrischen Matrix. Keine neue Information. Nicht berechnen oder speichern.
    end if:
    if i < 7 and j < 7 then # unterer linker Teil
      i_MM := index_symmat2vec(NQ,i,j): # Passender Index für zeilenweise ausgewählten symmetrischen Teil (siehe Gesamt-Massenmatrix)
      itmp := itmp + 1:
      MMbb_regressor_s[itmp,..] := MM_regressor_s[i_MM,..]:
    end if:
  end do:
end do:
if codeexport_inertia and not(base_method_name="twist") then
  MatlabExport(MMbb_regressor_s, sprintf("../codeexport/%s/inertia_base_base_floatb_%s_%s_matlab.m", robot_name, base_method_name, regressor_modus), codegen_opt):
end if:
# Mass Matrix Time Derivative
# Gesamt Massenmatrix
MM_regressor_t := convert_s_t(MM_regressor_s):
MMD_regressor_t := diff~(MM_regressor_t, t):
MMD_regressor_s := convert_t_s(MMD_regressor_t):
if codeexport_inertiaD and not(base_method_name="twist") then
  MatlabExport(MMD_regressor_s, sprintf("../codeexport/%s/inertiaD_floatb_%s_%s_matlab.m", robot_name, base_method_name, regressor_modus), codegen_opt):
end if:
# Konvertiere Gelenk-Massenmatrix in zeitabhängige Variablen, um Zeitableitung zu berechnen
MMjj_regressor_t := convert_s_t(MMjj_regressor_s):
MMDjj_regressor_t := diff~(MMjj_regressor_t, t):
MMDjj_regressor_s := convert_t_s(MMDjj_regressor_t):
if codeexport_inertiaD and base_method_name="twist" then
  MatlabExport(MMDjj_regressor_s, sprintf("../codeexport/%s/inertiaD_joint_joint_%s_%s_matlab.m", robot_name, expstring, regressor_modus), codegen_opt):
end if:
# Gelenk-Basis-Massenmatrix
MMjb_regressor_t := convert_s_t(MMjb_regressor_s):
MMDjb_regressor_t := diff~(MMjb_regressor_t, t):
MMDjb_regressor_s := convert_t_s(MMDjb_regressor_t):
if codeexport_inertiaD and not(base_method_name="twist") then
  MatlabExport(MMDjb_regressor_s, sprintf("../codeexport/%s/inertiaD_joint_base_floatb_%s_%s_matlab.m", robot_name, base_method_name, regressor_modus), codegen_opt):
end if:
# Coriolis Vector
# Generieren (gleiches Vorgehen für fixed und floating base)
tauC_regressor_s := dTdqDdt_s-dTdq_s:
for i to NQ do 
  tauC_regressor_s := subs({qDD_s(i, 1) = 0}, tauC_regressor_s):
end do:
save tauC_regressor_s, sprintf("../codeexport/%s/coriolisvec_joint_%s_%s_maple.m", robot_name, expstring, regressor_modus):
# Matlab Export
# Belastung der Gelenke
if codeexport_corvec then
  MatlabExport(tauC_regressor_s(7..NQ,..), sprintf("../codeexport/%s/coriolisvec_joint_%s_%s_matlab.m", robot_name, expstring, regressor_modus), codegen_opt):
end if:
# Gesamter Vektor für Floating Base
if codeexport_corvec and not(base_method_name="twist") then
  MatlabExport(tauC_regressor_s(1..NQ,..), sprintf("../codeexport/%s/coriolisvec_%s_%s_matlab.m", robot_name, expstring, regressor_modus), codegen_opt):
end if:
# Nur Basis-Terme
if codeexport_corvec and not(base_method_name="twist") then
  MatlabExport(tauC_regressor_s(1..6,..), sprintf("../codeexport/%s/coriolisvec_base_%s_%s_matlab.m", robot_name, expstring, regressor_modus), codegen_opt):
end if:
# Coriolis Matrix
# Floating-Base Coriolismatrix (Gesamt)
# Calculation with Christoffel Symbol approach
# [KhalilDombre2002], equ. (9.7) (p. 195)
# Funktion zur Anwendung des Algorithmus aus [KhalilDombre2002]:
cijk := proc (i::integer, j::integer, k::integer, A, qs)
  local c:
  c := (1/2)*(diff(A[i, j], qs(k, 1)))+(1/2)*(diff(A[i, k], qs(j, 1)))-(1/2)*(diff(A[j, k], qs(i, 1))):
  return c:
end proc:
# Initialisierung. Speichere die vollständige Coriolismatrix (nicht symmetrisch/schiefsymmetrisch). Unterschied zum Vorgehen bei der Massenmatrix
C_regressor_s := Matrix(NQ*NQ, Paramvec_size):
# Berechnung
i_rr := 0: # Vektor-Index für den Regressor der Coriolismatrix (Ausgabe)
for i to NQ do # Zeilenindex der Coriolismatrix
  for j to NQ do  # Spaltenindex der Coriolismatrix
    i_rr := i_rr + 1: # Gehe zeilenweise durch den unteren linken Teil der Coriolismatrix (inkl. Diagonale)
    for k from 1 to Paramvec_size do # Spaltenindex der Regressormatrix
      # Massenmatrix für Parameter k generieren (für Funktion mit Christoffel-Symbol-Ansatz benötigt)
      MM_k := Matrix(NQ,NQ):
      for ii from 1 to NQ do
        for jj from 1 to NQ do
          MM_k(ii,jj) := MM_regressor_s(index_symmat2vec(NQ, ii, jj), k):
        end do:
      end do:
      for l from 1 to NQ do # Siehe [KhalilDombre2002]
        C_regressor_s[i_rr, k] := C_regressor_s[i_rr,k] + cijk(i,j,l,MM_k,q_s)*qD_s[l,1]:
      end do:
    end do:
  end do:
end do:
save C_regressor_s, sprintf("../codeexport/%s/coriolismat_%s_%s_maple.m", robot_name, expstring, regressor_modus):
# Matlab Export: Floating Base Gesamt
if codeexport_cormat and not(base_method_name="twist") then
  MatlabExport(C_regressor_s, sprintf("../codeexport/%s/coriolismat_floatb_%s_%s_matlab.m", robot_name, base_method_name, regressor_modus), codegen_opt):
end if:
# Gelenk-Coriolismatrix
# Extrahiere Teilmatrix, die für die Gelenkterme (ohne Basiseinfluss) zuständig sind
Cjj_regressor_s := Matrix(NQJ*NQJ, Paramvec_size):
itmp := 0:
for i to NQ do # Zeilenindex der Coriolismatrix
  for j to NQ do  # Spaltenindex der Coriolismatrix
    if i > 6 and j > 6 then # unterer rechter Teil
      i_C := (i-1)*NQ+j: # Passender Index für zeilenweise ausgewähltes Element
      itmp := itmp + 1:
      Cjj_regressor_s[itmp,..] := C_regressor_s[i_C,..]:
    end if:
  end do:
end do:
if codeexport_cormat and base_method_name="twist" then
  MatlabExport(Cjj_regressor_s, sprintf("../codeexport/%s/coriolismat_joint_fixb_%s_matlab.m", robot_name, regressor_modus), codegen_opt):
end if:
# Inverse Dynamics
# Generate
tau_regressor_s := dTdqDdt_s-dTdq_s+dUdq_s:
save tau_regressor_s, sprintf("../codeexport/%s/invdyn_%s_%s_maple.m", robot_name, expstring, regressor_modus):
# Gesamter Vektor (floating base)
if codeexport_invdyn and not(base_method_name="twist") then
  MatlabExport(tau_regressor_s(1..NQ,..), sprintf("../codeexport/%s/invdyn_%s_%s_matlab.m", robot_name, expstring, regressor_modus), codegen_opt):
end if:
# Belastung der Basis (floating base)
if codeexport_invdyn and not(base_method_name="twist") then
  MatlabExport(tau_regressor_s(1..6,..), sprintf("../codeexport/%s/invdyn_base_%s_%s_matlab.m", robot_name, expstring, regressor_modus), codegen_opt):
end if:
# Belastung der Gelenke
if codeexport_invdyn then
  MatlabExport(tau_regressor_s(7..NQ,..), sprintf("../codeexport/%s/invdyn_joint_%s_%s_matlab.m", robot_name, expstring, regressor_modus), codegen_opt):
end if:

