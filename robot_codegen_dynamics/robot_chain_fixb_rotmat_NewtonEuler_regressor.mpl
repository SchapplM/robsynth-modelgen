
# Base Parameter Regressor for Robot based on MDH frames
# Einleitung
# Erstellung einer parameterlinearen Regressorform in Newton Euler Bewegungsgleichung
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# chain -> Berechnung für eine serielle Struktur (nicht: Baumstruktur)
# fixb -> fixed base. Kein Floating base Modell. Dort ist diese Form der Minimalparameterform nicht möglich.
# rotmat -> Kinematik wird mit Rotationsmatrizen berechnet
# NewtonEuler -> Berechnung der Newton Euler Bewegungsgleichung
# regressor -> Regressorform (parameterlinear)
# 
# Initialisierung
interface(warnlevel=0): # Unterdrücke die folgende Warnung.
restart: # Gibt eine Warnung, wenn über Terminal-Maple mit read gestartet wird.
interface(warnlevel=3):
interface(rtablesize=100): # Zur Anzeige von größeren Vektoren
;
with(LinearAlgebra):
with(ArrayTools):
with(codegen):
with(CodeGeneration):
with(StringTools):
codegen_act := true:
codegen_opt := 2:
codeexport_grav := true: 
codeexport_corvec := true:
codeexport_cormat := true:
codeexport_inertia := true:
codeexport_inertiaD := true:
codeexport_invdyn := true:
codeexport_act:= true:
codegen_dynpar := 2:
read "../helper/proc_convert_s_t":
read "../helper/proc_convert_t_s": 
read "../helper/proc_MatlabExport":
read "../helper/proc_simplify2":
read "../robot_codegen_definitions/robot_env":
printf("Generiere Regressorform für %s\n", robot_name, codegen_dynpar):
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name, base_method_name):
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
# Ergebnisse der Newton Euler- Bewegungsgleichung laden
read sprintf("../codeexport/%s/tmp/invdyn_%s_NewtonEuler_linkframe_par%d_maple.m", robot_name, base_method_name, codegen_dynpar):
f_i_i := f_i_i:
m_i_i := m_i_i:
tau_B := tau_B:
tau_J := tau_J:
# Mit diesem Arbeitsblatt werden die Vorwärtsrekursive für Fixed-Base Modelle generiert. Erkenne welche Basis-Modellierung aktiv ist
if base_method_name="twist" then # Basis-Methode "twist" wird (hier) nur für fixed Base benutzt
  expstring:="fixb":
elif base_method_name="eulxyz" then 
  expstring:="floatb_eulxyz":
else
  printf("Nicht behandelte Basis-Methode: %s\n", base_method_name):
end if:
# Die kinetischen und potentiellen Energien aus (2) und (3) stehen ab hier durch T_fixb und U_fixb zur Verfügung. 
# Der Parametervektor 'PV2_vec' aus (13) wurde in 'robot_tree_floatb_twist_definitions.mw' aufgestellt. 

# Parameterlinearisierung
# Parameterlinearisierung auf Basis von [HRL_IDR] (14) und (15)
# Linearisierung_Gelenkmomente
tauJ_regressor := Matrix(NQJ, 10*(NL-1)):

for i from 1 to NQJ do 
  for j from 1 to 10*(NL-1) do
    tauJ_regressor[i,j] :=  diff(tau_J(i,1),PV2_vec[10+j,1]):
  end do:
end do:
# Terme vereinfachen
if use_simplify>=1 then
  tmp_t1:=time():
  tmp_l1 := length(tauJ_regressor):
  tauJ_regressor := simplify2(tauJ_regressor):
  tmp_t2:=time():
  tmp_l2 := length(tauJ_regressor):
  printf("%s: Gelenkmoment-Regressor vereinfacht. Länge: %d->%d. Rechenzeit %1.1fs.\n", \
    FormatTime("%Y-%m-%d %H:%M:%S"), tmp_l1, tmp_l2, tmp_t2-tmp_t1):
end if:

# Linearisierung_Gelenkmomente(base)
# 
tauB_regressor := Matrix(6,10*NL):
for i from 1 to 6 do 
  for j from 1 to 10*NL do
    tauB_regressor[i,j] := diff(tau_B(i,1),PV2_vec[j,1]):
  end do:
end do:
# Terme vereinfachen
if use_simplify>=1 then
  tmp_t1:=time():
  tmp_l1 := length(tauB_regressor):
  tauB_regressor := simplify2(tauB_regressor):
  tmp_t2:=time():
  tmp_l2 := length(tauB_regressor):
  printf("%s: Basis-Kraft/Moment-Regressor vereinfacht. Länge: %d->%d. Rechenzeit %1.1fs.\n", \
    FormatTime("%Y-%m-%d %H:%M:%S"), tmp_l1, tmp_l2, tmp_t2-tmp_t1):
end if:


# Linearisierung_Schnittmomente
m_regressor := Matrix(3*NL,10*NL):

m_i_i_vec := Matrix(3*(NL),1):
for i from 1 to NL do
  for j from 1 to 3 do
    m_i_i_vec[3*(i-1)+j] := m_i_i[j,i]:
  end do:
end do:

for i from 1 to 3*NL do 
  for j from 1 to 10*NL do
    m_regressor[i,j] := diff~(m_i_i_vec(i,1),PV2_vec[j,1]):
  end do:
end do:

# Terme vereinfachen
if use_simplify>=1 then
  tmp_t1:=time():
  tmp_l1 := length(m_regressor):
  m_regressor := simplify2(m_regressor):
  tmp_t2:=time():
  tmp_l2 := length(m_regressor):
  printf("%s: Schnittmoment-Regressor vereinfacht. Länge: %d->%d. Rechenzeit %1.1fs.\n", \
    FormatTime("%Y-%m-%d %H:%M:%S"), tmp_l1, tmp_l2, tmp_t2-tmp_t1):
end if:

# Linearisierung_Schnittkräfte
f_regressor := Matrix(3*NL,10*NL):
f_i_i_vec := Matrix(3*(NL),1):
for i from 1 to NL do
  for j from 1 to 3 do 
    f_i_i_vec[3*(i-1)+j] := f_i_i[j,i]:
  end do:
end do:

for i from 1 to 3*NL do 
  for j from 1 to 10*NL do
    f_regressor[i,j] := diff~(f_i_i_vec(i,1),PV2_vec[j,1]):
  end do:
end do:
# Terme vereinfachen
if use_simplify>=1 then
  tmp_t1:=time():
  tmp_l1 := length(f_regressor):
  f_regressor := simplify2(f_regressor):
  tmp_t2:=time():
  tmp_l2 := length(f_regressor):
  printf("%s: Schnittkraft-Regressor vereinfacht. Länge: %d->%d. Rechenzeit %1.1fs.\n", \
    FormatTime("%Y-%m-%d %H:%M:%S"), tmp_l1, tmp_l2, tmp_t2-tmp_t1):
end if:

# Export
# Maple Export
save tauJ_regressor, sprintf("../codeexport/%s/tmp/fixb_NewtonEuler_tauJ_regressor_maple.m", robot_name):
save tauB_regressor, sprintf("../codeexport/%s/tmp/fixb_NewtonEuler_tauB_regressor_maple.m", robot_name):
save m_regressor, sprintf("../codeexport/%s/tmp/fixb_NewtonEuler_m_regressor_maple.m", robot_name):
save f_regressor, sprintf("../codeexport/%s/tmp/fixb_NewtonEuler_f_regressor_maple.m", robot_name):
printf("Maple-Ausdrücke exportiert. %s\n", FormatTime("%Y-%m-%d %H:%M:%S")):
# Matlab Export
if codegen_act then
  MatlabExport(convert_t_s(tauJ_regressor), sprintf("../codeexport/%s/tmp/invdyn_%s_NewtonEuler_tauJ_regressor_matlab.m", robot_name, expstring), codegen_opt):
  MatlabExport(convert_t_s(tauB_regressor), sprintf("../codeexport/%s/tmp/invdyn_%s_NewtonEuler_tauB_regressor_matlab.m", robot_name, expstring), codegen_opt):
  MatlabExport(convert_t_s(m_regressor), sprintf("../codeexport/%s/tmp/invdyn_%s_NewtonEuler_m_regressor_matlab.m", robot_name, expstring), codegen_opt):
  MatlabExport(convert_t_s(f_regressor), sprintf("../codeexport/%s/tmp/invdyn_%s_NewtonEuler_f_regressor_matlab.m", robot_name, expstring), codegen_opt):
end if
;

