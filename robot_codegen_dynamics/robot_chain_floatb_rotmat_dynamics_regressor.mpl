
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
codegen_act := true:
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
read "../helper/proc_simplify2":
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
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name):
read sprintf("../codeexport/%s/tmp/kinematic_constraints_maple_inert.m", robot_name):  
kin_constraints_exist := kin_constraints_exist: # nur zum Abschätzen der Komplexität
;
# Term-Vereinfachungen einstellen
if not assigned(simplify_options) or simplify_options(10)=-1 then # Standard-Einstellungen:
  if not kin_constraints_exist then # normale serielle Ketten und Baumstrukturen
    use_simplify := 0: # Standardmäßig aus
  else # mit kinematischen Zwangsbedingungen
    use_simplify := 1: # standardmäßig simplify-Befehle anwenden
  end if:
else # Benutzer-Einstellungen:
  use_simplify := simplify_options(10): # zehnter Eintrag ist für Dynamik-Regressor
end if:
# Mit diesem Arbeitsblatt werden die Regressor-Ausdrücke für Fixed-Base und Floating-Base Modelle generiert. Erkenne welche Basis-Modellierung aktiv ist
if base_method_name="twist" then # Basis-Methode "twist" wird (hier) nur für fixed Base benutzt
  expstring:="fixb":
elif base_method_name="eulxyz" then 
  expstring:="floatb_eulxyz":
else
  printf("Nicht behandelte Basis-Methode: %s\n", base_method_name):
fi:
# Es ist in diesem Arbeitsblatt möglich, zwei verschiedene Regressoren zu generieren und zu exportieren: Basierend auf Minimalparametern und auf vollem Parametersatz (PV2).
# Der Term "regressor" oder "regressor_minpar" ist jeweils in den Dateinamen enthalten.
# Der folgende Befehl muss immer auf "regressor_minpar" gesetzt sein, da diese Zeile durch das Skript robot_codegen_maple_preparation.sh ausgewertet und modifiziert wird.
regressor_modus := "regressor_minpar":
if regressor_modus = "regressor_minpar" then
  ekinfile := sprintf("../codeexport/%s/tmp/energy_kinetic_%s_regressor_minpar_maple.m", robot_name, expstring):
  if FileTools[Exists](ekinfile) then
    read ekinfile:
  else
    printf("%s. Energie-Regressor in Minimalparameterform wurde nicht berechnet. Abbruch der Dynamik-Minimalparameterregressor-Berechnung.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
    quit: # Funktioniert in GUI nicht richtig...
    robot_name := "": # ...Daher auch Löschung des Roboternamens.
  end if:
  read ekinfile:
  read sprintf("../codeexport/%s/tmp/energy_potential_%s_regressor_minpar_maple.m", robot_name, expstring):
  t_ges := t_ges_minpar:
  u_ges := u_ges_minpar:
  printf("%s. Generiere Minimalparameterregressor der Dynamik für %s\n", FormatTime("%Y-%m-%d %H:%M:%S"), robot_name):
end if:
if regressor_modus = "regressor" then
  read sprintf("../codeexport/%s/tmp/energy_kinetic_%s_regressor_maple.m", robot_name, expstring):
  read sprintf("../codeexport/%s/tmp/energy_potential_%s_regressor_maple.m", robot_name, expstring):
  t_ges := t_ges:
  u_ges := u_ges:
  printf("%s. Generiere Regressor der Dynamik für %s (nicht Minimalparameter)\n", FormatTime("%Y-%m-%d %H:%M:%S"), robot_name):
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
printf("%s. Generiere Dynamik-Regressor (%s) für %s und %s\n", FormatTime("%Y-%m-%d %H:%M:%S"), DynString, robot_name, base_method_name):
# Platzhalter-Vektor der Dynamik-Parameter aufstellen
nDP := ColumnDimension(u_ges):
PV := Matrix(nDP,1):
for i from 1 to nDP do
  if regressor_modus = "regressor_minpar" then
    PV(i,1) := parse(sprintf("MDP%d%", i)):
  elif regressor_modus = "regressor" then
    PV(i,1) := parse(sprintf("DP%d%", i)):
  end if:
end do:
PV:
# Name für Export der Terme als Funktion des Parametervektors
if regressor_modus = "regressor_minpar" then
  regshortname := "mdp":
elif regressor_modus = "regressor" then
  regshortname := "dp":
end if:

# Lagrange Formalismus (mit Funktion)
OutputLagrange := LagrangeN(t_ges, u_ges):
dTdqDdt_s := OutputLagrange[1]:
dTdq_s := OutputLagrange[2]:
dUdq_s := OutputLagrange[3]:
save dUdq_s, sprintf("../codeexport/%s/tmp/floatb_lagrange_dUdq_s_%s_maple.m", robot_name, regressor_modus):
save dTdq_s, sprintf("../codeexport/%s/tmp/floatb_lagrange_dTdq_s_%s_maple.m", robot_name, regressor_modus):
save dTdqDdt_s, sprintf("../codeexport/%s/tmp/floatb_lagrange_dTdqDdt_s_%s_maple.m", robot_name, regressor_modus):
# Extraktion einzelner Terme
# Gravitational Load
# Generate
taug_regressor_s := dUdq_s:
# Terme vereinfachen
if codeexport_grav then
  if use_simplify=1 then
    tmp_t1:=time():
    tmp_l1 := length(taug_regressor_s):
    taug_regressor_s := simplify2(taug_regressor_s):
    tmp_t2:=time():
    tmp_l2 := length(taug_regressor_s):
    printf("%s: Gravitationsmoment-Regressor vereinfacht. Länge: %d->%d. Rechenzeit %1.1fs.\n", \
      FormatTime("%Y-%m-%d %H:%M:%S"), tmp_l1, tmp_l2, tmp_t2-tmp_t1):
  end if:
end if:
save taug_regressor_s, sprintf("../codeexport/%s/tmp/gravload_%s_maple.m", robot_name, regressor_modus):
# Matlab Export
# Belastung der Basis (nur Floating Base)
if codegen_act and codeexport_grav and not(base_method_name="twist") then
  MatlabExport(taug_regressor_s(1..6,..), sprintf("../codeexport/%s/tmp/gravload_base_floatb_%s_%s_matlab.m", robot_name, base_method_name, regressor_modus), codegen_opt):
end if:
# Belastung der Gelenke: 
# Fixed-Base: Gravitationsvektor im Basis-KS
# Floating Base: Gravitationsvektor im Welt-KS, Basis-Orientierung berücksichtigt
taug := taug_regressor_s . PV:
if codegen_act and codeexport_grav then
  MatlabExport(taug_regressor_s(7..NQ,..), sprintf("../codeexport/%s/tmp/gravload_joint_%s_%s_matlab.m", robot_name, expstring, regressor_modus), codegen_opt):
  MatlabExport(taug(7..NQ,..), sprintf("../codeexport/%s/tmp/gravload_joint_%s_%s_matlab.m", robot_name, expstring, regshortname), codegen_opt):
end if:
# Kompletter Vektor
if codegen_act and codeexport_grav and not(base_method_name="twist") then
  MatlabExport(taug_regressor_s(1..NQ,..), sprintf("../codeexport/%s/tmp/gravload_floatb_%s_%s_matlab.m", robot_name, base_method_name, regressor_modus), codegen_opt):
  MatlabExport(taug(1..NQ,..), sprintf("../codeexport/%s/tmp/gravload_floatb_%s_%s_matlab.m", robot_name, expstring, regshortname), codegen_opt):
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
if codeexport_inertia or codeexport_inertiaD or codeexport_cormat then
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
# Terme vereinfachen
if use_simplify>=1 then
  tmp_t1:=time():
  tmp_l1 := length(MM_regressor_s):
  printf("%s. Beginne Vereinfachung: Massenmatrix-Regressor (%s). Länge: %d.\n", \
    FormatTime("%Y-%m-%d %H:%M:%S"), regressor_modus, tmp_l1):
  MM_regressor_s := simplify2(MM_regressor_s):
  tmp_t2:=time():
  tmp_l2 := length(MM_regressor_s):
  printf("%s. Massenmatrix-Regressor (%s) vereinfacht. Länge: %d->%d. Rechenzeit %1.1fs.\n", \
    FormatTime("%Y-%m-%d %H:%M:%S"), regressor_modus, tmp_l1, tmp_l2, tmp_t2-tmp_t1):
end if:
end if:
# Export Gesamt-Massenmatrix
if codegen_act and codeexport_inertia and not(base_method_name="twist") then
  MatlabExport(MM_regressor_s, sprintf("../codeexport/%s/tmp/inertia_floatb_%s_%s_matlab.m", robot_name, base_method_name, regressor_modus), codegen_opt):
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
# Multiplikation der Massenmatrix mit dem Regressorvektor und Export als obere Dreiecksmatrix
MMjj_tmp := MMjj_regressor_s . PV: # Vektor der oberen rechten Dreiecksmatrix
;
# Export Gelenkmassenmatrix
if codegen_act and codeexport_inertia and base_method_name="twist" then
  MatlabExport(MMjj_regressor_s, sprintf("../codeexport/%s/tmp/inertia_joint_joint_%s_%s_matlab.m", robot_name, expstring, regressor_modus), codegen_opt):
  MatlabExport(MMjj_tmp, sprintf("../codeexport/%s/tmp/inertia_joint_joint_%s_%s_matlab.m", robot_name, expstring, regshortname), codegen_opt):
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
if codegen_act and codeexport_inertia and not(base_method_name="twist") then
  MatlabExport(MMjb_regressor_s, sprintf("../codeexport/%s/tmp/inertia_joint_base_floatb_%s_%s_matlab.m", robot_name, base_method_name, regressor_modus), codegen_opt):
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
if codegen_act and codeexport_inertia and not(base_method_name="twist") then
  MatlabExport(MMbb_regressor_s, sprintf("../codeexport/%s/tmp/inertia_base_base_floatb_%s_%s_matlab.m", robot_name, base_method_name, regressor_modus), codegen_opt):
end if:
# Mass Matrix Time Derivative
# Gesamt Massenmatrix
MM_regressor_t := convert_s_t(MM_regressor_s):
MMD_regressor_t := diff~(MM_regressor_t, t):
MMD_regressor_s := convert_t_s(MMD_regressor_t):
if codegen_act and codeexport_inertiaD and not(base_method_name="twist") then
  MatlabExport(MMD_regressor_s, sprintf("../codeexport/%s/tmp/inertiaD_floatb_%s_%s_matlab.m", robot_name, base_method_name, regressor_modus), codegen_opt):
end if:
# Konvertiere Gelenk-Massenmatrix in zeitabhängige Variablen, um Zeitableitung zu berechnen
MMjj_regressor_t := convert_s_t(MMjj_regressor_s):
MMDjj_regressor_t := diff~(MMjj_regressor_t, t):
MMDjj_regressor_s := convert_t_s(MMDjj_regressor_t):
if codegen_act and codeexport_inertiaD and base_method_name="twist" then
  MatlabExport(MMDjj_regressor_s, sprintf("../codeexport/%s/tmp/inertiaD_joint_joint_%s_%s_matlab.m", robot_name, expstring, regressor_modus), codegen_opt):
end if:
# Gelenk-Basis-Massenmatrix
MMjb_regressor_t := convert_s_t(MMjb_regressor_s):
MMDjb_regressor_t := diff~(MMjb_regressor_t, t):
MMDjb_regressor_s := convert_t_s(MMDjb_regressor_t):
if codegen_act and codeexport_inertiaD and not(base_method_name="twist") then
  MatlabExport(MMDjb_regressor_s, sprintf("../codeexport/%s/tmp/inertiaD_joint_base_floatb_%s_%s_matlab.m", robot_name, base_method_name, regressor_modus), codegen_opt):
end if:
# Coriolis Vector
# Generieren (gleiches Vorgehen für fixed und floating base)
tauC_regressor_s := dTdqDdt_s-dTdq_s:
for i to NQ do 
  tauC_regressor_s := subs({qDD_s(i, 1) = 0}, tauC_regressor_s):
end do:
save tauC_regressor_s, sprintf("../codeexport/%s/tmp/coriolisvec_joint_%s_%s_maple.m", robot_name, expstring, regressor_modus):
tauc := tauC_regressor_s . PV:
# Matlab Export
# Belastung der Gelenke
if codegen_act and codeexport_corvec then
  MatlabExport(tauC_regressor_s(7..NQ,..), sprintf("../codeexport/%s/tmp/coriolisvec_joint_%s_%s_matlab.m", robot_name, expstring, regressor_modus), codegen_opt):
  MatlabExport(tauc(7..NQ,..), sprintf("../codeexport/%s/tmp/coriolisvec_joint_%s_%s_matlab.m", robot_name, expstring, regshortname), codegen_opt):
end if:
# Gesamter Vektor für Floating Base
if codegen_act and codeexport_corvec and not(base_method_name="twist") then
  MatlabExport(tauC_regressor_s(1..NQ,..), sprintf("../codeexport/%s/tmp/coriolisvec_%s_%s_matlab.m", robot_name, expstring, regressor_modus), codegen_opt):
  MatlabExport(tauc(1..NQ,..), sprintf("../codeexport/%s/tmp/coriolisvec_%s_%s_matlab.m", robot_name, expstring, regshortname), codegen_opt):
end if:
# Nur Basis-Terme
if codegen_act and codeexport_corvec and not(base_method_name="twist") then
  MatlabExport(tauC_regressor_s(1..6,..), sprintf("../codeexport/%s/tmp/coriolisvec_base_%s_%s_matlab.m", robot_name, expstring, regressor_modus), codegen_opt):
  MatlabExport(tauc(1..6,..), sprintf("../codeexport/%s/tmp/coriolisvec_base_%s_%s_matlab.m", robot_name, expstring, regshortname), codegen_opt):
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
if codeexport_cormat then:
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
# Terme vereinfachen
if use_simplify>=1 then
  tmp_t1:=time():
  tmp_l1 := length(C_regressor_s):
  printf("%s. Beginne Vereinfachung: Coriolis-Matrix-Regressor (%s). Länge: %d.\n", \
    FormatTime("%Y-%m-%d %H:%M:%S"), regressor_modus, tmp_l1):
  C_regressor_s := simplify2(C_regressor_s):
  tmp_t2:=time():
  tmp_l2 := length(C_regressor_s):
  printf("%s: Coriolis-Matrix-Regressor (%s) vereinfacht. Länge: %d->%d. Rechenzeit %1.1fs.\n", \
    FormatTime("%Y-%m-%d %H:%M:%S"), regressor_modus, tmp_l1, tmp_l2, tmp_t2-tmp_t1):
end if:
end if:
save C_regressor_s, sprintf("../codeexport/%s/tmp/coriolismat_%s_%s_maple.m", robot_name, expstring, regressor_modus):
# Matlab Export: Floating Base Gesamt
if codegen_act and codeexport_cormat and not(base_method_name="twist") then
  MatlabExport(C_regressor_s, sprintf("../codeexport/%s/tmp/coriolismat_floatb_%s_%s_matlab.m", robot_name, base_method_name, regressor_modus), codegen_opt):
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
if codegen_act and codeexport_cormat and base_method_name="twist" then
  MatlabExport(Cjj_regressor_s, sprintf("../codeexport/%s/tmp/coriolismat_joint_fixb_%s_matlab.m", robot_name, regressor_modus), codegen_opt):
end if:
# Inverse Dynamics
# Generate
tau_regressor_s := dTdqDdt_s-dTdq_s+dUdq_s:
save tau_regressor_s, MMjj_regressor_s, tauMM_regressor_s, tauC_regressor_s, taug_regressor_s, sprintf("../codeexport/%s/tmp/invdyn_%s_%s_maple.m", robot_name, expstring, regressor_modus):
tau := tau_regressor_s . PV:
# Gesamter Vektor (floating base)
if codegen_act and codeexport_invdyn and not(base_method_name="twist") then
  MatlabExport(tau_regressor_s(1..NQ,..), sprintf("../codeexport/%s/tmp/invdyn_%s_%s_matlab.m", robot_name, expstring, regressor_modus), codegen_opt):
  MatlabExport(tau, sprintf("../codeexport/%s/tmp/invdyn_%s_%s_matlab.m", robot_name, expstring, regshortname), codegen_opt):
end if:
# Belastung der Basis (floating base)
if codegen_act and codeexport_invdyn and not(base_method_name="twist") then
  MatlabExport(tau_regressor_s(1..6,..), sprintf("../codeexport/%s/tmp/invdyn_base_%s_%s_matlab.m", robot_name, expstring, regressor_modus), codegen_opt):
  MatlabExport(tau(1..6,..), sprintf("../codeexport/%s/tmp/invdyn_base_%s_%s_matlab.m", robot_name, expstring, regshortname), codegen_opt):
end if:
# Belastung der Gelenke
if codegen_act and codeexport_invdyn then
  MatlabExport(tau_regressor_s(7..NQ,..), sprintf("../codeexport/%s/tmp/invdyn_joint_%s_%s_matlab.m", robot_name, expstring, regressor_modus), codegen_opt):
  MatlabExport(tau(7..NQ,..), sprintf("../codeexport/%s/tmp/invdyn_joint_%s_%s_matlab.m", robot_name, expstring, regshortname), codegen_opt):
end if:
# Regressor-Matrix als Vektor hinschreiben. Dabei Null-Einträge herausfinden und nicht weiter berücksichtigen.
# Dadurch wird die obere rechte Dreiecksform der Regressor-Matrix ausgenutzt.
# Platzhalter-Vektor für alle Einträge der Regressor-Matrix
Reg_Vector := Matrix(ColumnDimension(tau_regressor_s)*RowDimension(tau_regressor_s), 1):
# Gelenkmoment in Abhängigkeit des Regressor-Vektors (RV)
tauJ_RV := Matrix(NQJ, 1):
kk := 0: # Zähler für laufende Nummer des Regressor-Vektors
for i from 1 to ColumnDimension(tau_regressor_s) do
  for j from 1 to RowDimension(tau_regressor_s)-6 do
    # Gehe Spalten- und dann Zeilenweise durch die Regressormatrix (nur Gelenk-Teil)
    if tau_regressor_s(j+6,i) <> 0 then # Eintrag ungleich Null
    	 kk := kk + 1:
    	 # Regressor-Vektor-Eintrag als Platzhalter direkt so, wie er in Matlab-Funktion gebraucht wird ("RV" ist Eingabe der Matlab-Funktion).
    	 rv_kk := parse(sprintf("RV(%d)", kk)):
    	 # Eintrag für Regressor-Vektor so schreiben, dass er in einer Matlab-Funktion aus der Regressor-Matrix gebildet werden kann
    	 Reg_Vector(kk,1) := parse(sprintf("RM(%d,%d)", j, i)):
    	 # Gelenkmoment aus Multiplikation mit Parametervektor. Dadurch Berechnung unter Ausnutzung der Nullen.
    	 tauJ_RV(j) := tauJ_RV(j) + rv_kk*PV(i):
    end if:
  end do:
end do:
# Kürzen des Regressor-Vektors auf die tatsächlich benutzten Einträge
Reg_Vector := Reg_Vector(1..kk,..):
save Reg_Vector, sprintf("../codeexport/%s/tmp/invdyn_joint_%s_%s_occupancy_vector_maple", robot_name, expstring, regressor_modus):
if codegen_act and codeexport_invdyn then
  interface(warnlevel=0): # Unterdrücke die folgende Warnung (Ist prinzipbedingt, da RM/RV als Funktionsnamen interpretiert werden. Der Code funktioniert trotzdem.
  MatlabExport(Reg_Vector, sprintf("../codeexport/%s/tmp/invdyn_joint_%s_%s_occupancy_vector_matlab.m", robot_name, expstring, regressor_modus), codegen_opt):
  MatlabExport(tauJ_RV, sprintf("../codeexport/%s/tmp/invdyn_joint_%s_%s_mult_matlab.m", robot_name, expstring, regshortname), codegen_opt):
  interface(warnlevel=3):
end if:

