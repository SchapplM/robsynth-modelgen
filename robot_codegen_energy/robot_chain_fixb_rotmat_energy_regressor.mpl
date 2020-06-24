
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
# (C) Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# Quellen
# [HRL_IDR] Skript Humanoid Robotics Lab - Serial Chain Robot Identification
# [GautierKhalil1988] A Direct Determination of Minimum Inertial Parameters of Robots
# [GautierKhalil1990] Direct Calculation of Minimum Set of Inertial Parameters of Serial Robots
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
read "../helper/proc_convert_s_t":
read "../helper/proc_convert_t_s": 
read "../helper/proc_MatlabExport":
read "../helper/proc_simplify2":
read "../robot_codegen_definitions/robot_env":
printf("%s. Generiere Minimalparameterregressor der Energie für %s (geometrischer Ansatz)\n", \
  FormatTime("%Y-%m-%d %H:%M:%S"), robot_name, codegen_dynpar):
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name, base_method_name):
read sprintf("../codeexport/%s/tmp/kinematic_constraints_maple_inert.m", robot_name):  
kin_constraints_exist := kin_constraints_exist: # nur zum Abschätzen der Komplexität
;
# Term-Vereinfachungen einstellen
if not assigned(simplify_options) or simplify_options(7)=-1 then # Standard-Einstellungen:
  if not kin_constraints_exist then # normale serielle Ketten und Baumstrukturen
    use_simplify := 0: # Standardmäßig aus
  else # mit kinematischen Zwangsbedingungen
    use_simplify := 1: # standardmäßig simplify-Befehle anwenden
  end if:
else # Benutzer-Einstellungen:
  use_simplify := simplify_options(7): # siebter Eintrag ist für Energie-Regressor
end if:

# Ergebnisse der Energie laden
read sprintf("../codeexport/%s/tmp/energy_potential_floatb_%s_worldframe_par2_maple.m", robot_name, base_method_name):
read sprintf("../codeexport/%s/tmp/energy_kinetic_floatb_%s_linkframe_par2_maple.m", robot_name, base_method_name):
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
t_ges := Matrix(1, 10*(NL-1)):
u_ges := Matrix(1, 10*(NL-1)):
for i to 10*(NL-1) do 
  t_ges[1,i] := diff(T_fixb,PV2_vec[10+i,1]);
  u_ges[1,i] := diff(U_fixb,PV2_vec[10+i,1]);
end do:
# Terme vereinfachen
if use_simplify=1 then
  tmp_t1:=time():
  tmp_l11 := length(t_ges):
  tmp_l12 := length(u_ges):
  t_ges := simplify2(t_ges):
  u_ges := simplify2(u_ges):
  tmp_t2:=time():
  tmp_l21 := length(t_ges):
  tmp_l22 := length(u_ges):
  printf("%s: Terme für Energie-Regressor vereinfacht. Länge: %d->%d / %d->%d. Rechenzeit %1.1fs.\n", \
    FormatTime("%Y-%m-%d %H:%M:%S"), tmp_l11, tmp_l21, tmp_l12, tmp_l22, tmp_t2-tmp_t1):
end if:
# Export
save t_ges, sprintf("../codeexport/%s/tmp/energy_kinetic_fixb_regressor_maple.m", robot_name):
save u_ges, sprintf("../codeexport/%s/tmp/energy_potential_fixb_regressor_maple.m", robot_name):
if codegen_act then
  MatlabExport(convert_t_s(t_ges), sprintf("../codeexport/%s/tmp/energy_kinetic_fixb_regressor_matlab.m", robot_name), codegen_opt):
end if:
if codegen_act then
  MatlabExport(convert_t_s(u_ges), sprintf("../codeexport/%s/tmp/energy_potential_fixb_regressor_matlab.m", robot_name), codegen_opt):
end if:
# Parameterminimierung
# Prüfe, ob der folgende Algorithmus funktionieren kann
# Bestimme, ob es eine Baumstruktur ist. Wenn ja, funktioniert der Algorithmus (noch) nicht (nicht implementiert).
tree:=false:
for i from 1 to NJ do
  if not v(i) = i-1 then
    tree := true: break:
  end if:
end do:

if assigned(user_CoM) or assigned(user_M) or assigned(user_inertia) \ 
  or kin_constraints_exist or tree then
  # es gibt einen Sonderfall, diese Berechnung der Minimalparameter funktioniert voraussichtlich nicht.
  printf("%s. Geometrische Berechnung der Minimalparameter nicht möglich.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  quit: # Funktioniert in GUI nicht richtig...
  robot_name := "": # ...Daher auch Löschung des Roboternamens.
end if:
# Minimalparametervekor
# Definiere Parametermatrix
# Nehme nur die Inertialparameter der bewegten Segmente, nicht die Basis.
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
# [GautierKhalil1990] (eq. 15,16)
for i from NJ by -1 to 2 do
  # printf("i=%d, sigma=%d\n", i, sigma[i,1]): 
  if sigma[i,1] = 0 then # Drehgelenk (eq. 15)
    XX[i,  1] := XX[i,  1]-YY[i,1]:
    XX[i-1,1] := XX[i-1,1] + YY[i,1] + d[i,1]^2*m[i,1]+2*d[i,1]*mZ[i,1]: 
    XY[i-1,1] := XY[i-1,1] + a[i,1]*sin(alpha[i,1])*mZ[i,1]+a[i,1]*d[i,1]*sin(alpha[i,1])*m[i,1]: 
    XZ[i-1,1] := XZ[i-1,1] - a[i,1]*cos(alpha[i,1])*mZ[i,1]-a[i,1]*d[i,1]*cos(alpha[i,1])*m[i,1]: 
    YY[i-1,1] := YY[i-1,1] + cos(alpha[i,1])^2*YY[i,1]+2*d[i,1]*cos(alpha[i,1])^2*mZ[i,1]+(a[i,1]^2+d[i,1]^2*cos(alpha[i,1])^2)*m[i,1]:
    YZ[i-1,1] := YZ[i-1,1] + cos(alpha[i,1])*sin(alpha[i,1])*YY[i,1]+2*d[i,1]*cos(alpha[i,1])*sin(alpha[i,1])*mZ[i,1]+d[i,1]^2*cos(alpha[i,1])*sin(alpha[i,1])*m[i,1]:
    ZZ[i-1,1] := ZZ[i-1,1] + sin(alpha[i,1])^2*YY[i,1]+2*d[i,1]*sin(alpha[i,1])^2*mZ[i,1]+(a[i,1]^2+d[i,1]^2*sin(alpha[i,1])^2)*m[i,1]:
    mX[i-1,1] := mX[i-1,1] + a[i,1]*m[i,1]: 
    mY[i-1,1] := mY[i-1,1] - sin(alpha[i,1])*mZ[i,1]-d[i,1]*sin(alpha[i,1])*m[i,1]:
    mZ[i-1,1] := mZ[i-1,1] + cos(alpha[i,1])*mZ[i,1]+d[i,1]*cos(alpha[i,1])*m[i,1]:
    m[i-1 ,1] := m[i-1 ,1] + m[i,1]:
  else: # Schubgelenk (eq. 16)
    XX[i-1,1] := XX[i-1,1] + cos(theta[i,1])^2*XX[i,1]
                           - 2*cos(theta[i,1])*sin(theta[i,1])*XY[i,1]+sin(theta[i,1])^2*YY[i,1]: 
    XY[i-1,1] := XY[i-1,1] + cos(theta[i,1])*sin(theta[i,1])*cos(alpha[i,1])*XX[i,1]
                           + (cos(theta[i,1])^2-sin(theta[i,1])^2)*cos(alpha[i,1])*XY[i,1]
                           - cos(theta[i,1])*sin(alpha[i,1])*XZ[i,1]
                           - cos(theta[i,1])*sin(theta[i,1])*cos(alpha[i,1])*YY[i,1]
                           + sin(theta[i,1])*sin(alpha[i,1])*YZ[i,1]:
    XZ[i-1,1] := XZ[i-1,1] + cos(theta[i,1])*sin(theta[i,1])*sin(alpha[i,1])*XX[i,1]
                           + (cos(theta[i,1])^2-sin(theta[i,1])^2)*sin(alpha[i,1])*XY[i,1]
                           + cos(theta[i,1])*cos(alpha[i,1])*XZ[i,1]
                           - cos(theta[i,1])*sin(theta[i,1])*sin(alpha[i,1])*YY[i,1]
                           - sin(theta[i,1])*cos(alpha[i,1])*YZ[i,1]:
    YY[i-1,1] := YY[i-1,1] + sin(theta[i,1])^2*cos(alpha[i,1])^2*XX[i,1]
                           + 2*cos(theta[i,1])*sin(theta[i,1])*cos(alpha[i,1])^2*XY[i,1]
                           - 2*sin(theta[i,1])*cos(alpha[i,1])*sin(alpha[i,1])*XZ[i,1]
                           + cos(theta[i,1])^2*cos(alpha[i,1])^2*YY[i,1]
                           - 2*cos(theta[i,1])*cos(alpha[i,1])*sin(alpha[i,1])*YZ[i,1]
                           + sin(alpha[i,1])^2*ZZ[i,1]:
    YZ[i-1,1] := YZ[i-1,1] + sin(theta[i,1])^2*cos(alpha[i,1])*sin(alpha[i,1])*XX[i,1]
                           + 2*cos(theta[i,1])*sin(theta[i,1])*cos(alpha[i,1])*sin(alpha[i,1])*XY[i,1]
                           + sin(theta[i,1])*(cos(alpha[i,1])^2-sin(alpha[i,1])^2)*XZ[i,1]
                           + cos(theta[i,1])^2*cos(alpha[i,1])*sin(alpha[i,1])*YY[i,1]
                           + cos(theta[i,1])*(cos(alpha[i,1])^2-sin(alpha[i,1])^2)*YZ[i,1]
                           - cos(alpha[i,1])*sin(alpha[i,1])*ZZ[i,1]:
    ZZ[i-1,1] := ZZ[i-1,1] + sin(theta[i,1])^2*sin(alpha[i,1])^2*XX[i,1]
                           + 2*cos(theta[i,1])*sin(theta[i,1])*sin(alpha[i,1])^2*XY[i,1]
                           + 2*sin(theta[i,1])*cos(alpha[i,1])*sin(alpha[i,1])*XZ[i,1]
                           + cos(theta[i,1])^2*sin(alpha[i,1])^2*YY[i,1]
                           + 2*cos(theta[i,1])*cos(alpha[i,1])*sin(alpha[i,1])*YZ[i,1]
                           + cos(alpha[i,1])^2*ZZ[i,1]:
    # Sonderfall parallele Achsen (eq. 19)
    if sin(alpha[i,1]) = 0 then
      ZZ[i-1,1] := ZZ[i-1,1] + 2*a[i,1]*cos(theta[i,1])*mX[i,1] - 2*a[i,1]*sin(theta[i,1])*mY[i,1]:
      # TODO: Wird ZZ hier nicht doppelt regruppiert?
      mX[i-1,1] := mX[i-1,1] + cos(theta[i,1])*mX[i,1] - sin(theta[i,1])*mY[i,1]:
      mY[i-1,1] := mY[i-1,1] + sin(theta[i,1])*cos(alpha[i,1])*mX[i,1] + cos(theta[i,1])*cos(alpha[i,1])*mY[i,1]:
    end if:
    # Sonderfall senkrechte Achsen (eq. 20)
    if cos(alpha[i,1]) = 0 then
      # YY[i-1,1] := YY[i-1,1] + 2*a[i,1]*cos(theta[i,1])*mX[i,1] - 2*a[i,1]*sin(theta[i,1])*mY[i,1]:
      # TODO: Wird YY hier nicht doppelt regruppiert?
      # mX[i-1,1] := mX[i-1,1] + cos(theta[i,1])*mX[i,1] - sin(theta[i,1])*mY[i,1]:
      # mZ[i-1,1] := mZ[i-1,1] + sin(alpha[i,1])*sin(theta[i,1])*mX[i,1] + sin(alpha[i,1])*cos(theta[i,1])*mY[i,1]:
      # funktioniert irgendwie noch nicht. TODO!
    end if:
  end if:
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
    for j to NQJ do
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

       if `mod`(i, 10) = 1 then XX[trunc((1/10)*i)+1, 1] := 0; printf("XX%d \n", trunc((1/10)*i)+1): end if:
       if `mod`(i, 10) = 2 then XY[trunc((1/10)*i)+1, 1] := 0; printf("XY%d \n", trunc((1/10)*i)+1): end if:
       if `mod`(i, 10) = 3 then XZ[trunc((1/10)*i)+1, 1] := 0; printf("XZ%d \n", trunc((1/10)*i)+1): end if:
       if `mod`(i, 10) = 4 then YY[trunc((1/10)*i)+1, 1] := 0; printf("YY%d \n", trunc((1/10)*i)+1): end if:
       if `mod`(i, 10) = 5 then YZ[trunc((1/10)*i)+1, 1] := 0; printf("YZ%d \n", trunc((1/10)*i)+1): end if:
       if `mod`(i, 10) = 6 then ZZ[trunc((1/10)*i)+1, 1] := 0; printf("ZZ%d \n", trunc((1/10)*i)+1): end if:
       if `mod`(i, 10) = 7 then mX[trunc((1/10)*i)+1, 1] := 0; printf("MX%d \n", trunc((1/10)*i)+1): end if:
       if `mod`(i, 10) = 8 then mY[trunc((1/10)*i)+1, 1] := 0; printf("MY%d \n", trunc((1/10)*i)+1): end if:
       if `mod`(i, 10) = 9 then mZ[trunc((1/10)*i)+1, 1] := 0; printf("MZ%d \n", trunc((1/10)*i)+1): end if:
       if `mod`(i, 10) = 0 then m[trunc((1/10)*i), 1] := 0;    printf(" M%d \n", trunc((1/10)*i)):   end if:
  end if:
end do:
# Speichere markierte Regressor-Vektoren
save t_ges, sprintf("../codeexport/%s/tmp/energy_kinetic_fixb_regressor_marked_maple.m", robot_name):
save u_ges, sprintf("../codeexport/%s/tmp/energy_potential_fixb_regressor_marked_maple.m", robot_name):
# Parametervektor in Richtiger Reihenfolge aufstellen und Entfernen von mZ, YY, m
# Anwendung von [HRL_IDR] Regel 3 (S. 5) auf den Parametervektor
# Bei Drehgelenken werden die Parameter YY, mZ und m mit denen des folgenden Körpers gruppiert. Bei Schubgelenken werden alle Trägheitsmomente gruppiert (da die Trägheitsmomente keinen Einfluss auf die translatorische Bewegung des Schubgelenkes haben).
# [GautierKhalil1990] (eq. 18)
nt:=add(sigma[i,1],i=1..NJ): # Anzahl Translatorische Gelenke
nr:=NJ-nt: # Anzahl rotatorische Gelenke
;
#Anzahl der Einträge im Minimalparametervektor
MPV_n_max := 7*nr+4*nt: # ignoriere Abzüge in der Formel
;
Paramvec := Matrix(MPV_n_max, 1):
ii := 1:
for i to NJ do
  if sigma[i,1] = 0 then # Drehgelenk (eq. 15)
    Paramvec[ii, 1] := XX[i, 1]: ii:= ii+1:
    Paramvec[ii, 1] := XY[i, 1]: ii:= ii+1:
    Paramvec[ii, 1] := XZ[i, 1]: ii:= ii+1:
    Paramvec[ii, 1] := YZ[i, 1]: ii:= ii+1:
    Paramvec[ii, 1] := ZZ[i, 1]: ii:= ii+1:
    Paramvec[ii, 1] := mX[i, 1]: ii:= ii+1:
    Paramvec[ii, 1] := mY[i, 1]: ii:= ii+1:
  else: # Schubgelenk (eq. 16)
    if not sin(alpha[i,1]) = 0 then
      # Sonderregel senkrechte Achsen
      # [GautierKhalil1990] (eq. 19). Parameter wurden mit denen der Basisnäheren Achse zusammengelegt und werden hier nicht betrachtet.
      Paramvec[ii, 1] := mX[i, 1]: ii:= ii+1:
      Paramvec[ii, 1] := mY[i, 1]: ii:= ii+1:
    end if:
    if not cos(alpha[i,1]) = 0 then
      # Sonderregel parallele Achsen
      # [GautierKhalil1990] (eq. 20).
      # funktioniert irgendwie noch nicht. TODO!
    end if:
    Paramvec[ii, 1] := mZ[i, 1]: ii:= ii+1:
    Paramvec[ii, 1] := m[i, 1]:  ii:= ii+1:
  end if:
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
Paramvec2;
# Export - Minimalparametervektor
save Paramvec2, sprintf("../codeexport/%s/tmp/minimal_parameter_vector_fixb_maple", robot_name):
save Paramvec2, sprintf("../codeexport/%s/tmp/minimal_parameter_vector_fixb_maple_geom", robot_name): # zum Testen gegen andere Implementierung
if codegen_act then
   MatlabExport(Paramvec2, sprintf("../codeexport/%s/tmp/minimal_parameter_vector_fixb_matlab.m", robot_name), codegen_opt):
end if;
# Minimal geometrievektor t_i und u_i
# Markieren von t_mZ, t_YY, t_m, u_mZ, u_YY, u_m_j
# Anwendung von [HRL_IDR] Regel 3 (S. 5) auf die Geometrievektoren t und u der Energien T und U
# Der Basis-Körper wird nicht gezählt. Daher ignorieren der ersten zehn Einträge in t_ges
# [GautierKhalil1990], p.369 (rechts: "As a conclusion ..."
for i from 0 to NJ-1 do 
  if sigma[i+1,1] = 0 then: # Drehgelenk
    t_ges[1,i*10+ 4]:=REMOVE; # YY
    t_ges[1,i*10+ 9]:=REMOVE; # mZ
    t_ges[1,i*10+10]:=REMOVE; # m
    u_ges[1,i*10+ 4]:=REMOVE; # YY
    u_ges[1,i*10+ 9]:=REMOVE; # mZ
    u_ges[1,i*10+10]:=REMOVE; # m
  else: # Schubgelenk
    u_ges[1,i*10+ 1]:=REMOVE; # XX
    u_ges[1,i*10+ 2]:=REMOVE; # XY
    u_ges[1,i*10+ 3]:=REMOVE; # XZ
    u_ges[1,i*10+ 4]:=REMOVE; # YY
    u_ges[1,i*10+ 5]:=REMOVE; # YZ
    u_ges[1,i*10+ 6]:=REMOVE; # ZZ
    t_ges[1,i*10+ 1]:=REMOVE; # XX
    t_ges[1,i*10+ 2]:=REMOVE; # XY
    t_ges[1,i*10+ 3]:=REMOVE; # XZ
    t_ges[1,i*10+ 4]:=REMOVE; # YY
    t_ges[1,i*10+ 5]:=REMOVE; # YZ
    t_ges[1,i*10+ 6]:=REMOVE; # ZZ
    if sin(alpha[i+1,1]) = 0 then
      # Sonderregel senkrechte Achsen
      # [GautierKhalil1990] (eq. 19). Parameter zusammengelegt in Parametervektor. Werden daher hier entfernt.
     u_ges[1,i*10+ 7]:=REMOVE; # mX
     u_ges[1,i*10+ 8]:=REMOVE; # mY
     t_ges[1,i*10+ 7]:=REMOVE; # mX
     t_ges[1,i*10+ 8]:=REMOVE; # mY
    end if:
    if cos(alpha[i+1,1]) = 0 then
      # Sonderregel parallele Achsen
      # [GautierKhalil1990] (eq. 20). 
    end if:
  end if:
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
save t_ges_minpar, sprintf("../codeexport/%s/tmp/energy_kinetic_fixb_regressor_minpar_maple.m", robot_name):
save u_ges_minpar, sprintf("../codeexport/%s/tmp/energy_potential_fixb_regressor_minpar_maple.m", robot_name):
# Zum symbolischen Testen des minimierten Regressors gegen den ursprünglichen
(*
read sprintf("../codeexport/%s/tmp/energy_kinetic_fixb_regressor_maple.m", robot_name):
t_ges := t_ges: # Variable wurde im Arbeitsblatt verändert. Neu laden.
read sprintf("../codeexport/%s/tmp/energy_potential_fixb_regressor_maple.m", robot_name):
u_ges := u_ges:
# Muss Null sein.
test_u := simplify(u_ges_minpar.Paramvec2 - u_ges.PV2_vec(11..,..)):
test_t := simplify(t_ges_minpar.Paramvec2 - t_ges.PV2_vec(11..,..)):
*)
;
# Export
if codegen_act then
  MatlabExport(convert_t_s(t_ges_minpar), sprintf("../codeexport/%s/tmp/energy_kinetic_fixb_regressor_minpar_matlab.m", robot_name), codegen_opt):
end if:
if codegen_act then
  MatlabExport(convert_t_s(u_ges_minpar), sprintf("../codeexport/%s/tmp/energy_potential_fixb_regressor_minpar_matlab.m", robot_name), codegen_opt):
end if:


