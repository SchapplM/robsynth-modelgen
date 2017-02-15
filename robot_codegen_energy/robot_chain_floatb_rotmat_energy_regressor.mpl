# Base Parameter Energy Regressor for Robot based on MDH frames
# Einleitung
# Erstellung einer parameterlinearen und -minimalen Regressorform
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# chain -> Berechnung für eine serielle Struktur (nicht: Baumstruktur)
# floatb -> fixed base. Kein Floating base Modell. Dort ist diese Form der Minimalparameterform nicht möglich.
# rotmat -> Kinematik wird mit Rotationsmatrizen berechnet
# energy -> Berechnung der Energie
# regressor -> Regressorform (parameterlinear)
# Authors
# Alexander Tödtheide, toedtheide@irt.uni-hannover.de
# Moritz Schappler, schappler@irt.uni-hannover.de, 2017-02
# 
# Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# Quellen
# [HRL_IDR] Skript Humanoid Robotics Lab - Serial Chain Robot Identification
# [GautierKhalil1988] A Direct Determination of Minimum Inertial Parameters of Robots
# [GautierKhalil1990] Direct Calculation of Minimum Set of Inertial Parameters of Serial Robots
# Initialisierung
interface(warnlevel=0): # Unterdrücke die folgende Warnung.
restart: # Gibt eine Warnung, wenn über Terminal-Maple mit read gestartet wird.
interface(warnlevel=3):
with(LinearAlgebra):
with(ArrayTools):
with(codegen):
with(CodeGeneration):
with(StringTools):
codegen_act := true:
codegen_opt := 2:
read "../helper/proc_convert_s_t":
read "../helper/proc_convert_t_s": 
read "../helper/proc_MatlabExport":
read "../robot_codegen_definitions/robot_env":
printf("Generiere Minimalparameterregressor (Floating Base) der Energie für %s\n", robot_name, codegen_dynpar):
read sprintf("../codeexport/%s/tree_floatb_definitions", robot_name, base_method_name):
# Ergebnisse der Energie laden
read sprintf("../codeexport/%s/energy_potential_floatb_%s_worldframe_par2_maple.m", robot_name, base_method_name):
read sprintf("../codeexport/%s/energy_kinetic_floatb_%s_linkframe_par2_maple.m", robot_name, base_method_name):
T_floatb := T:
U_floatb := U_grav:
# Die kinetischen und potentiellen Energien aus (2) und (3) stehen ab hier durch T_floatb und U_floatb zur Verfügung. 
# Der Parametervektor 'PV2_vec' aus (13) wurde in 'robot_tree_floatb_twist_definitions.mw' aufgestellt. 
# Parameterlinearisierung
# Parameterlinearisierung auf Basis von [HRL_IDR] (14) und (15)
# Linearisierung
t_ges := Matrix(1, 10*NL):
u_ges := Matrix(1, 10*NL):
for i to 10*NL do 
  t_ges[1,i] := diff(T_floatb,PV2_vec[i,1]);
  u_ges[1,i] := diff(U_floatb,PV2_vec[i,1]);
end do: 
# Export
save t_ges, sprintf("../codeexport/%s/energy_kinetic_floatb_%s_regressor_maple.m", robot_name, base_method_name):
save u_ges, sprintf("../codeexport/%s/energy_potential_floatb_%s_regressor_maple.m", robot_name, base_method_name):
if codegen_act then
  MatlabExport(convert_t_s(t_ges), sprintf("../codeexport/%s/energy_kinetic_floatb_%s_regressor_matlab.m", robot_name, base_method_name), codegen_opt):
end if:
if codegen_act then
  MatlabExport(convert_t_s(u_ges), sprintf("../codeexport/%s/energy_potential_floatb_%s_regressor_matlab.m", robot_name, base_method_name), codegen_opt):
end if:
# Parameterminimierung
# Minimalparametervekor
# Definiere Parametermatrix
XX := Matrix(NL, 1, PV2_mat[1 .. NL, 1]):
XY := Matrix(NL, 1, PV2_mat[1 .. NL, 2]):
XZ := Matrix(NL, 1, PV2_mat[1 .. NL, 3]):
YY := Matrix(NL, 1, PV2_mat[1 .. NL, 4]):
YZ := Matrix(NL, 1, PV2_mat[1 .. NL, 5]):
ZZ := Matrix(NL, 1, PV2_mat[1 .. NL, 6]):
mX := Matrix(NL, 1, PV2_mat[1 .. NL, 7]):
mY := Matrix(NL, 1, PV2_mat[1 .. NL, 8]):
mZ := Matrix(NL, 1, PV2_mat[1 .. NL, 9]):
m :=  Matrix(NL, 1, PV2_mat[1 .. NL, 10]):
# Rekursive Berechnung der Minimalparameter
# Rekursive Berechnung der Minimalparameter mit [HRL_IDR] (23), siehe Abbildung 1.
# [GautierKhalil1990] (eq. 15)
for i from NL by -1 to 2 do 
  j := i-1: # Laufvariable für die MDH-Kinematikparameter (Inertialparameter von Segment 0 fangen auch bei 1 an).
  XX[i,1] := XX[i,1]-YY[i,1]:
  XX[i-1,1] := d[j,1]^2*m[i,1]+2*d[j,1]*mZ[i,1]+XX[i-1,1]+YY[i,1]: 
  XY[i-1,1] := XY[i-1,1]+a[j,1]*sin(alpha[j,1])*mZ[i,1]+a[j,1]*d[j,1]*sin(alpha[j,1])*m[i,1]: 
  XZ[i-1,1] := XZ[i-1,1]-a[j,1]*cos(alpha[j,1])*mZ[i,1]-a[j,1]*d[j,1]*cos(alpha[j,1])*m[i,1]: 
  YY[i-1,1] := YY[i-1,1]+cos(alpha[j,1])^2*YY[i,1]+2*d[j,1]*cos(alpha[j,1])^2*mZ[i,1]+(a[j,1]^2+d[j,1]^2*cos(alpha[j,1])^2)*m[i,1]:
  YZ[i-1,1] := YZ[i-1,1]+cos(alpha[j,1])*sin(alpha[j,1])*YY[i,1]+2*d[j,1]*cos(alpha[j,1])*sin(alpha[j,1])*mZ[i,1]+d[j,1]^2*cos(alpha[j,1])*sin(alpha[j,1])*m[i,1]:
  ZZ[i-1,1] := ZZ[i-1,1]+sin(alpha[j,1])^2*YY[i,1]+2*d[j,1]*sin(alpha[j,1])^2*mZ[i,1]+(a[j,1]^2+d[j,1]^2*sin(alpha[j,1])^2)*m[i,1]:
  mX[i-1,1] := a[j,1]*m[i,1]+mX[i-1,1]: 
  mY[i-1,1] := mY[i-1,1]-sin(alpha[j,1])*mZ[i,1]-d[j,1]*sin(alpha[j,1])*m[i,1]:
  mZ[i-1,1] := mZ[i-1,1]+cos(alpha[j,1])*mZ[i,1]+d[j,1]*cos(alpha[j,1])*m[i,1]:
  m[i-1,1] := m[i-1,1]+m[i,1]:
end do: 
# Parameter ohne Einfluss "Markieren"
# Es kann keine Parameter mehr ohne Einfluss geben, da die Basis-Bewegung die Trägheit aller Segmente anregt.
# Parametervektor in Richtiger Reihenfolge aufstellen und Entfernen von mZ, YY, m
# Anwendung von [HRL_IDR] Regel 3 (S. 5) auf den Parametervektor
# [GautierKhalil1990] (eq. 18)
MPV_n_max := 10+7*NJ: # ignoriere Abzüge in der Formel
Paramvec := Matrix(MPV_n_max, 1):
# Basis-Parameter direkt übernehmen
Paramvec(1..10,1) := <XX[1,1],XY[1,1],XZ[1,1],YY[1,1],YZ[1,1],ZZ[1,1],mX[1,1],mY[1,1],mZ[1,1],m[1,1]>:
# Paramvec(1..10,1);
# Formel für Parameter, die durch Gelenke bewegt werden, anwenden
for i from 1 to NJ do 
  Paramvec[10+7*(i-1)+1, 1] := XX[i+1, 1]:
  Paramvec[10+7*(i-1)+2, 1] := XY[i+1, 1]:
  Paramvec[10+7*(i-1)+3, 1] := XZ[i+1, 1]:
  Paramvec[10+7*(i-1)+4, 1] := YZ[i+1, 1]:
  Paramvec[10+7*(i-1)+5, 1] := ZZ[i+1, 1]:
  Paramvec[10+7*(i-1)+6, 1] := mX[i+1, 1]:
  Paramvec[10+7*(i-1)+7, 1] := mY[i+1, 1]:
end do: 
# Entfernung von Nullelementen
# Durch Entfernung der Nullelemente erfolgt 'beta_b' aus [HRL_IDR] (22)
p := 0:
for i to MPV_n_max do 
  if Paramvec[i, 1] = 0 then 
    p := p+1 
  end if:
end do:
# Determine size of matrix dependent on number of Zeroes
Paramvec_size := MPV_n_max-p:
Paramvec2 := Matrix(Paramvec_size, 1):
p := 0:
for i to MPV_n_max do 
  # count Zeroes
  if Paramvec[i, 1] = 0 then  
    p := p+1:
  else 
    # Shift by number of currently found Zeroes
    Paramvec2[i-p, 1] := Paramvec[i, 1]:
  end if:
end do: 
printf("Dimension des Minimalparametervektors: %dx1\n", Paramvec_size):
# Export - Minimalparametervektor
save Paramvec2, sprintf("../codeexport/%s/minimal_parameter_vector_floatb_%s_maple", robot_name, base_method_name):
if codegen_act then
   MatlabExport(Paramvec2, sprintf("../codeexport/%s/minimal_parameter_vector_floatb_%s_matlab.m", robot_name, base_method_name), codegen_opt):
end if;
# Minimal geometrievektor t_i und u_i
# Markieren von t_mZ, t_YY, t_m, u_mZ, u_YY, u_m_j
# Anwendung von [HRL_IDR] Regel 3 (S. 5) auf die Geometrievektoren t und u der Energien T und U
# Die Regel bezieht sich auf  Körper, die durch Drehgelenke bewegt werden. Einträge für den Basiskörper werden daher nicht geändert.
# Die Parameter YY,MZ und M sind immer linear abhängig. Die entsprechenden Spalten der Regressormatrizen werden daher zum Entfernen markiert.
# [GautierKhalil1990], p.369
for i from 1 to NL-1 do 
  t_ges[1,i*10+ 4]:=REMOVE; #YY
  t_ges[1,i*10+ 9]:=REMOVE; #MZ
  t_ges[1,i*10+10]:=REMOVE; #M

  u_ges[1,i*10+ 4]:=REMOVE; #YY
  u_ges[1,i*10+ 9]:=REMOVE; #MZ
  u_ges[1,i*10+10]:=REMOVE; #M

end do: 
# Entfernungen von markierten Elementen
# Durch Entfernung der markierten Elemente, äquivalent zum Minimalparametervektor beta_b, haben t und u die gleiche Spaltenanzahl wie 'C_b'.
p:=0:
for i from 1 to 10*NL do       #Nullen Zählen um die Matrixgröße von t_ges zu bestimmen   
  if t_ges[1,i]=REMOVE and u_ges[1,i]=REMOVE then       
    p:=p+1;
  end if
end do: 
size_Matrix:=10*NL-p: 
t_ges_minpar:=Matrix(1,size_Matrix):
u_ges_minpar:=Matrix(1,size_Matrix):
printf("Dimension der Minimalparameter-Regressormatrix: 1x%d\n", size_Matrix):

p:=0:
for i from 1 to 10*NL do       #Nullen Zählen     
   if t_ges[1,i]=REMOVE and u_ges[1,i]=REMOVE then       
      p:=p+1; 
   else 
      t_ges_minpar[1,i-p]:=t_ges[1,i]; #Um die Anzahl der im Iterationsschritt gezählten Nullen verschieben
      u_ges_minpar[1,i-p]:=u_ges[1,i]; #Um die Anzahl der im Iterationsschritt gezählten Nullen verschieben
   end if
end do: 
save t_ges_minpar, sprintf("../codeexport/%s/energy_kinetic_floatb_%s_regressor_minpar_maple.m", robot_name, base_method_name):
save u_ges_minpar, sprintf("../codeexport/%s/energy_potential_floatb_%s_regressor_minpar_maple.m", robot_name, base_method_name):
# Export
if codegen_act then
  MatlabExport(convert_t_s(t_ges_minpar), sprintf("../codeexport/%s/energy_kinetic_floatb_%s_regressor_minpar_matlab.m", robot_name, base_method_name), codegen_opt):
end if:
if codegen_act then
  MatlabExport(convert_t_s(u_ges_minpar), sprintf("../codeexport/%s/energy_potential_floatb_%s_regressor_minpar_matlab.m", robot_name, base_method_name), codegen_opt):
end if:

