# Base Parameter Energy Regressor for Robot based on MDH frames
# Einleitung
# Erstellung einer parameterlinearen und -minimalen Regressorform
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# chain -> Berechnung für eine serielle Struktur (nicht: Baumstruktur)
# fixb -> fixed base. Kein Floating base Modell. Dort ist diese Form der Minimalparameterform nicht möglich.
# rotmat -> Kinematik wird mit Rotationsmatrizen berechnet
# energy -> Berechnung der Energie
# regressor -> Regressorform (parameterlinear)
# Authors
# Alexander Tödtheide, toedtheide@irt.uni-hannover.de
# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
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
printf("Generiere Minimalparameterregressor der Energie für %s\n", robot_name, codegen_dynpar):
read sprintf("../codeexport/%s/tree_floatb_definitions", robot_name, base_method_name):
# Ergebnisse der Energie laden
read sprintf("../codeexport/%s/energy_potential_floatb_%s_worldframe_par2_maple.m", robot_name, base_method_name):
read sprintf("../codeexport/%s/energy_kinetic_floatb_%s_linkframe_par2_maple.m", robot_name, base_method_name):
T_floatb := T:
U_floatb := U_grav:
# Einfluss der Basis-Geschwindigkeit und -Position entfernen
T_fixb:=T_floatb:
U_fixb:=U_floatb:
for i from 1 to 6 do
  T_fixb := subs({V_base_t[i,1]=0},T_fixb):
  U_fixb := subs({X_base_t[i,1]=0},U_fixb):
end do:
# Die kinetischen und potentiellen Energien aus (2) und (3) stehen ab hier durch T_fixb und U_fixb zur Verfügung. 
# Der Parametervektor 'PV2_vec' aus (13) wurde in 'robot_tree_floatb_twist_definitions.mw' aufgestellt. 
# Parameterlinearisierung
# Parameterlinearisierung auf Basis von [HRL_IDR] (14) und (15)
# Linearisierung
t_ges := Matrix(1, 10*NJ):
u_ges := Matrix(1, 10*NJ):
for i to 10*NJ do 
  t_ges[1,i] := diff(T_fixb,PV2_vec[10+i,1]);
  u_ges[1,i] := diff(U_fixb,PV2_vec[10+i,1]);
end do: 
# Export
save t_ges, sprintf("../codeexport/%s/energy_kinetic_fixb_regressor_maple.m", robot_name):
save u_ges, sprintf("../codeexport/%s/energy_potential_fixb_regressor_maple.m", robot_name):
if codegen_act then
  MatlabExport(convert_t_s(t_ges), sprintf("../codeexport/%s/energy_kinetic_fixb_regressor_matlab.m", robot_name), codegen_opt):
end if:
if codegen_act then
  MatlabExport(convert_t_s(u_ges), sprintf("../codeexport/%s/energy_potential_fixb_regressor_matlab.m", robot_name), codegen_opt):
end if:

# Parameterminimierung
# Minimalparametervekor
# Definiere Parametermatrix
XX := Matrix(NJ, 1, PV2_mat[2 .. NL, 1]):
XY := Matrix(NJ, 1, PV2_mat[2 .. NL, 2]):
XZ := Matrix(NJ, 1, PV2_mat[2 .. NL, 3]):
YY := Matrix(NJ, 1, PV2_mat[2 .. NL, 4]):
YZ := Matrix(NJ, 1, PV2_mat[2 .. NL, 5]):
ZZ := Matrix(NJ, 1, PV2_mat[2 .. NL, 6]):
mX := Matrix(NJ, 1, PV2_mat[2 .. NL, 7]):
mY := Matrix(NJ, 1, PV2_mat[2 .. NL, 8]):
mZ := Matrix(NJ, 1, PV2_mat[2 .. NL, 9]):
m :=  Matrix(NJ, 1, PV2_mat[2 .. NL, 10]):
# Rekursive Berechnung der Minimalparameter
# Rekursive Berechnung der Minimalparameter mit [HRL_IDR] (23), siehe Abbildung 1.
# [GautierKhalil1990] (eq. 15)
for i from NJ by -1 to 2 do 
  XX[i,1] := XX[i,1]-YY[i,1]:
  XX[i-1,1] := d[i,1]^2*m[i,1]+2*d[i,1]*mZ[i,1]+XX[i-1,1]+YY[i,1]: 
  XY[i-1,1] := XY[i-1,1]+a[i,1]*sin(alpha[i,1])*mZ[i,1]+a[i,1]*d[i,1]*sin(alpha[i,1])*m[i,1]: 
  XZ[i-1,1] := XZ[i-1,1]-a[i,1]*cos(alpha[i,1])*mZ[i,1]-a[i,1]*d[i,1]*cos(alpha[i,1])*m[i,1]: 
  YY[i-1,1] := YY[i-1,1]+cos(alpha[i,1])^2*YY[i,1]+2*d[i,1]*cos(alpha[i,1])^2*mZ[i,1]+(a[i,1]^2+d[i,1]^2*cos(alpha[i,1])^2)*m[i,1]:
  YZ[i-1,1] := YZ[i-1,1]+cos(alpha[i,1])*sin(alpha[i,1])*YY[i,1]+2*d[i,1]*cos(alpha[i,1])*sin(alpha[i,1])*mZ[i,1]+d[i,1]^2*cos(alpha[i,1])*sin(alpha[i,1])*m[i,1]:
  ZZ[i-1,1] := ZZ[i-1,1]+sin(alpha[i,1])^2*YY[i,1]+2*d[i,1]*sin(alpha[i,1])^2*mZ[i,1]+(a[i,1]^2+d[i,1]^2*sin(alpha[i,1])^2)*m[i,1]:
  mX[i-1,1] := a[i,1]*m[i,1]+mX[i-1,1]: 
  mY[i-1,1] := mY[i-1,1]-sin(alpha[i,1])*mZ[i,1]-d[i,1]*sin(alpha[i,1])*m[i,1]:
  mZ[i-1,1] := mZ[i-1,1]+cos(alpha[i,1])*mZ[i,1]+d[i,1]*cos(alpha[i,1])*m[i,1]:
  m[i-1,1] := m[i-1,1]+m[i,1]:
end do: 
# Parameter ohne Einfluss "Markieren"
# Markierung von Parametern ohne Einfluss an Hand von [HRL_IDR] (18)
printf("The following parameters don't have any effect:\n"):

for i to 10*NJ do 
  effect := 0:
  if not (t_ges[1,i]=0) then
    # printf("t_ges[1,%d] not Null\n", i):
    effect := 1:
  else
    for j to NJ do
      if has(u_ges[1,i],{qJ_t(j,1)}) then
        # printf("u_ges[1,%d] not constant\n", i):
        effect := 1:
        break:
      end if:
    end do:
  end if:
  if effect = 0 then
       t_ges[1,i] := REMOVE:
       u_ges[1,i] := REMOVE:

       if `mod`(i, 10) = 1 then XX[trunc((1/10)*i)+1, 1] := 0; printf("XX%d \n", trunc((1/10)*i)+1+1) end if:
       if `mod`(i, 10) = 2 then XY[trunc((1/10)*i)+1, 1] := 0; printf("XY%d \n", trunc((1/10)*i)+1+1) end if:
       if `mod`(i, 10) = 3 then XZ[trunc((1/10)*i)+1, 1] := 0; printf("XZ%d \n", trunc((1/10)*i)+1+1) end if:
       if `mod`(i, 10) = 4 then YY[trunc((1/10)*i)+1, 1] := 0; printf("YY%d \n", trunc((1/10)*i)+1+1) end if:
       if `mod`(i, 10) = 5 then YZ[trunc((1/10)*i)+1, 1] := 0; printf("YZ%d \n", trunc((1/10)*i)+1+1) end if:
       if `mod`(i, 10) = 6 then ZZ[trunc((1/10)*i)+1, 1] := 0; printf("ZZ%d \n", trunc((1/10)*i)+1+1) end if:
       if `mod`(i, 10) = 7 then mX[trunc((1/10)*i)+1, 1] := 0; printf("mX%d \n", trunc((1/10)*i)+1+1) end if:
       if `mod`(i, 10) = 8 then mY[trunc((1/10)*i)+1, 1] := 0; printf("mY%d \n", trunc((1/10)*i)+1+1) end if:
       if `mod`(i, 10) = 9 then mZ[trunc((1/10)*i)+1, 1] := 0; printf("mZ%d \n", trunc((1/10)*i)+1+1) end if:
       if `mod`(i, 10) = 0 then m[trunc((1/10)*i), 1] := 0;    printf(" m%d \n", trunc((1/10)*i+1))   end if:
  end if:
end do:
# Parametervektor in Richtiger Reihenfolge aufstellen und Entfernen von mZ, YY, m
# Anwendung von [HRL_IDR] Regel 3 (S. 5) auf den Parametervektor
# [GautierKhalil1990] (eq. 18)
MPV_n_max := 7*NJ: # ignoriere Abzüge in der Formel
Paramvec := Matrix(MPV_n_max, 1):
for i to NJ do 
  Paramvec[7*(i-1)+1, 1] := XX[i, 1]:
  Paramvec[7*(i-1)+2, 1] := XY[i, 1]:
  Paramvec[7*(i-1)+3, 1] := XZ[i, 1]:
  Paramvec[7*(i-1)+4, 1] := YZ[i, 1]:
  Paramvec[7*(i-1)+5, 1] := ZZ[i, 1]:
  Paramvec[7*(i-1)+6, 1] := mX[i, 1]:
  Paramvec[7*(i-1)+7, 1] := mY[i, 1]:
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
save Paramvec2, sprintf("../codeexport/%s/minimal_parameter_vector_maple", robot_name):
if codegen_act then
   MatlabExport(Paramvec2, sprintf("../codeexport/%s/minimal_parameter_vector_matlab.m", robot_name), codegen_opt):
end if;
# Minimal geometrievektor t_i und u_i
# Markieren von t_mZ, t_YY, t_m, u_mZ, u_YY, u_m_j
# Anwendung von [HRL_IDR] Regel 3 (S. 5) auf die Geometrievektoren t und u der Energien T und U
# Der Basis-Körper wird nicht gezählt. Daher ignorieren der ersten zehn Einträge in t_ges
# [GautierKhalil1990], p.369
for i from 0 to NJ-1 do 

   t_ges[1,i*10+ 4]:=REMOVE;
   t_ges[1,i*10+ 9]:=REMOVE;
   t_ges[1,i*10+10]:=REMOVE;

   u_ges[1,i*10+ 4]:=REMOVE;
   u_ges[1,i*10+ 9]:=REMOVE;
   u_ges[1,i*10+10]:=REMOVE;

end do: 
# Entfernungen von markierten Elementen
# Durch Entfernung der markierten Elemente, äquivalent zum Minimalparametervektor beta_b, haben t und u die gleiche Spaltenanzahl wie 'C_b'.
p:=0:
for i from 1 to 10*NJ do       #Nullen Zählen um die Matrixgröße von t_ges zu bestimmen   
  if t_ges[1,i]=REMOVE and u_ges[1,i]=REMOVE then       
    p:=p+1;
  end if
end do: 
size_Matrix:=10*NJ-p: 
t_ges_minpar:=Matrix(1,size_Matrix):
u_ges_minpar:=Matrix(1,size_Matrix):
printf("Dimension der Regressormatrix: 1x%d\n", size_Matrix):

p:=0:
for i from 1 to 10*NJ do       #Nullen Zählen     
   if t_ges[1,i]=REMOVE and u_ges[1,i]=REMOVE then       
      p:=p+1; 
   else 
      t_ges_minpar[1,i-p]:=t_ges[1,i]; #Um die Anzahl der im Iterationsschritt gezählten Nullen verschieben
      u_ges_minpar[1,i-p]:=u_ges[1,i]; #Um die Anzahl der im Iterationsschritt gezählten Nullen verschieben
   end if
end do: 
save t_ges_minpar, sprintf("../codeexport/%s/energy_kinetic_fixb_regressor_minpar_maple.m", robot_name):
save u_ges_minpar, sprintf("../codeexport/%s/energy_potential_fixb_regressor_minpar_maple.m", robot_name):
# Export
if codegen_act then
  MatlabExport(convert_t_s(t_ges_minpar), sprintf("../codeexport/%s/energy_kinetic_fixb_regressor_minpar_matlab.m", robot_name), codegen_opt):
end if:
if codegen_act then
  MatlabExport(convert_t_s(u_ges_minpar), sprintf("../codeexport/%s/energy_potential_fixb_regressor_minpar_matlab.m", robot_name), codegen_opt):
end if:

