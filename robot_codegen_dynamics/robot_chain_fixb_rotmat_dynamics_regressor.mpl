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
# Autor
# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# Sources
# [GautierKhalil1990] Direct Calculation of Minimum Set of Inertial Parameters of Serial Robots
# [KhalilDombre2002] Modeling, Identification and Control of Robots
# [Ortmaier2014] Vorlesungsskript Robotik I
# Initialization
restart:
with(LinearAlgebra):
with(ArrayTools):
with(codegen):
with(CodeGeneration):
with(StringTools):
codegen_opt := 2:
codegen_act := true:
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
read sprintf("../codeexport/%s_tree_floatb_twist_definitions", robot_name):
# Es ist in diesem Arbeitsblatt möglich, zwei verschiedene Regressoren zu generieren und zu exportieren: Basierend auf Minimalparametern und auf vollem Parametersatz (PV2).
# Der Term "regressor" oder "regressor_minpar" ist jeweils in den Dateinamen enthalten
regressor_modus := "regressor_minpar":
if regressor_modus = "regressor_minpar" then
  read sprintf("../codeexport/%s_energy_kinetic_fixb_regressor_minpar_maple.m", robot_name):
  read sprintf("../codeexport/%s_energy_potential_fixb_regressor_minpar_maple.m", robot_name):
  t_ges := t_ges_minpar:
  u_ges := u_ges_minpar:
  printf("Generiere Minimalparameterregressor der Dynamik für %s\n", robot_name):
end if:
if regressor_modus = "regressor" then
  read sprintf("../codeexport/%s_energy_kinetic_fixb_regressor_maple.m", robot_name):
  read sprintf("../codeexport/%s_energy_potential_fixb_regressor_maple.m", robot_name):
  t_ges := t_ges:
  u_ges := u_ges:
  printf("Generiere Regressor der Dynamik für %s (nicht Minimalparameter)\n", robot_name):
end if:
# Lagrange Formalismus (mit Funktion)
OutputLagrange := LagrangeN(t_ges, u_ges):
dTdqDdt_s := OutputLagrange[1]:
dTdq_s := OutputLagrange[2]:
dUdq_s := OutputLagrange[3]:
save dUdq_s, sprintf("../codeexport/%s_floatb_lagrange_dUdq_s_%s_maple.m", robot_name, regressor_modus):
save dTdq_s, sprintf("../codeexport/%s_floatb_lagrange_dTdq_s_%s_maple.m", robot_name, regressor_modus):
save dTdqDdt_s, sprintf("../codeexport/%s_floatb_lagrange_dTdqDdt_s_%s_maple.m", robot_name, regressor_modus):
# Extraktion einzelner Terme
# Gravitational Load
# Generate
taug_regressor_s := dUdq_s:
save taug_regressor_s, sprintf("../codeexport/%s_gravload_%s_maple.m", robot_name, regressor_modus):
# Matlab Export
# Belastung der Gelenke
if codegen_act then
  MatlabExport(taug_regressor_s(7..NQ,..), sprintf("../codeexport/%s_gravload_joint_%s_matlab.m", robot_name, regressor_modus), codegen_opt):
end if:
# Mass Matrix
# Initialisiere. Speichere nur den unteren linken Teil der Massenmatrix
# Siehe: https://de.wikipedia.org/wiki/Symmetrische_Matrix
Paramvec_size := ColumnDimension(t_ges):
tauMMj_regressor_s := dTdqDdt_s(7..NQ,..):
MMjj_regressor_s := Matrix(NQJ*(NQJ+1)/2, Paramvec_size):
# Berechnung
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
# Matlab Export
if codegen_act then
  MatlabExport(MMjj_regressor_s, sprintf("../codeexport/%s_inertia_joint_joint_%s_matlab.m", robot_name, regressor_modus), codegen_opt):
end if:
# Mass Matrix Time Derivative
# Konvertiere Massenmatrix in zeitabhängige Variablen, um Zeitableitung zu berechnen
MMjj_regressor_t := convert_s_t(MMjj_regressor_s):
MMDjj_regressor_t := diff~(MMjj_regressor_t, t):
MMDjj_regressor_s := convert_t_s(MMDjj_regressor_t):
# Matlab Export
if codegen_act then
  MatlabExport(MMDjj_regressor_s, sprintf("../codeexport/%s_inertiaD_joint_joint_%s_matlab.m", robot_name, regressor_modus), codegen_opt):
end if:
# Coriolis Vector
# Generate
tauC_regressor_s := dTdqDdt_s-dTdq_s:
for i to NQ do 
  tauC_regressor_s := subs({qDD_s(i, 1) = 0}, tauC_regressor_s):
end do:
save tauC_regressor_s, sprintf("../codeexport/%s_coriolisvec_joint_fixb_%s_maple.m", robot_name, regressor_modus):
# Matlab Export
# Belastung der Gelenke
if codegen_act then
  MatlabExport(tauC_regressor_s(7..NQ,..), sprintf("../codeexport/%s_coriolisvec_joint_fixb_%s_matlab.m", robot_name, regressor_modus), codegen_opt):
end if:
# Coriolis Matrix
# Calculation with Christoffel Symbol approach
# [KhalilDombre2002], equ. (9.7) (p. 195)
# Funktion zur Anwendung des Algorithmus aus [KhalilDombre2002]:
cijk := proc (i::integer, j::integer, k::integer, A, qs)
  local c:
  c := (1/2)*(diff(A[i, j], q_s(k, 1)))+(1/2)*(diff(A[i, k], q_s(j, 1)))-(1/2)*(diff(A[j, k], q_s(i, 1))):
  return c:
end proc:
# Initialisierung. Speichere nur den unteren linken Teil der Matrix als Regressorform (siehe Vorgehen bei der Massenmatrix)
Cjj_regressor_s := Matrix(NQJ*(NQJ+1)/2, Paramvec_size):
# Berechnung
i_rr := 0: # Vektor-Index für den Regressor der Coriolismatrix (Ausgabe)
for i to NQJ do # Zeilenindex der Coriolismatrix
  for j to NQJ do  # Spaltenindex der Coriolismatrix
    if j > i then
      next: # rechte obere Seite der symmetrischen Matrix. Keine neue Information. Nicht berechnen oder speichern.
    end if:
    i_rr := i_rr + 1: # Gehe zeilenweise durch den unteren linken Teil der Coriolismatrix (inkl. Diagonale)
    for k to Paramvec_size do # Spaltenindex der Regressormatrix
      # Massenmatrix für Parameter k generieren (für Funktion mit Christoffel-Symbol-Ansatz benötigt)
      MM_jj_k := Matrix(NQJ,NQJ):
      for ii from 1 to NQJ do
        for jj from 1 to NQJ do
          MM_jj_k := MMjj_regressor_s(index_symmat2vec(NQJ, ii, jj), k):
        end do:
      end do:
      for l from 1 to NQJ do # Siehe [KhalilDombre2002]
        Cjj_regressor_s[i_rr, k] := Cjj_regressor_s[i_rr, k]+cijk(i,j,l,MM_jj_k,qJ_s)*qJD_s[l,1]:
      end do:
    end do:
  end do:
end do:
# Matlab Export: Fixed base
if codegen_act then
  MatlabExport(Cjj_regressor_s, sprintf("../codeexport/%s_coriolismat_joint_fixb_%s_matlab.m", robot_name, regressor_modus), codegen_opt):
end if:

# Inverse Dynamics
# Generate
tau_regressor_s := dTdqDdt_s-dTdq_s+dUdq_s:
save tau_regressor_s, sprintf("../codeexport/%s_invdyn_joint_fixb_%s_maple.m", robot_name, regressor_modus):
# Matlab Export
# Belastung der Gelenke
if codegen_act then
  MatlabExport(tau_regressor_s(7..NQ,..), sprintf("../codeexport/%s_invdyn_joint_fixb_%s_matlab.m", robot_name, regressor_modus), codegen_opt):
end if:

