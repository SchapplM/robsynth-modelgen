
# Inverse Dynamics for Parallel Robot Platform
# Einleitung
# Berechnung der inversen Dynamik der Roboter-Plattform
# 
# Dateiname:
# robot -> Berechnung f�r allgemeinen Roboter
# para -> Berechnung f�r eine parallelen Roboter
# plattform -> Bezogen auf Plattform der PKM
# rotmat -> Kinematik wird mit Rotationsmatrizen berechnet
# dynamics -> Berechnung der Dynamik
# Autor
# Tim Job (Studienarbeit bei Moritz Schappler), 2018-12
# Moritz Schappler, moritz.schappler@imes.uni-hannover.de
# (C) Institut f�r Mechatronische Systeme, Universit�t Hannover
# Sources
# [Abdellatif2007] Modellierung, Identifikation und robuste Regelung von Robotern mit parallelkinematischen Strukturen
# Initialization
interface(warnlevel=0): # Unterdr�cke die folgende Warnung.
restart: # Gibt eine Warnung, wenn �ber Terminal-Maple mit read gestartet wird.
interface(warnlevel=3):
with(LinearAlgebra):
with(codegen):
with(CodeGeneration):
with(StringTools):
with(VectorCalculus):
# Dynamik-Parametersatz (1=baryzentrisch, 2=inertial). Muss im Repo auf 1 bleiben, da der Wert per Bash-Skript ge�ndert wird.
codegen_dynpar := 1:
# Einstellungen f�r Code-Export: Optimierungsgrad (2=h�chster) und Aktivierung jedes Terms.
codegen_opt := 2:
codeexport_invdyn := false:
codeexport_grav := false:
codeexport_corvec := false:
codeexport_inertia := false:
read "../helper/proc_MatlabExport":
read "../helper/proc_vec2skew":
read "../helper/proc_vector2symmat":
read "../robot_codegen_definitions/robot_env_par":
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", leg_name):
# Definitionen f�r parallelen Roboter laden
read "../robot_codegen_definitions/robot_env_par":
read sprintf("../codeexport/%s/tmp/para_definitions", robot_name):
r_P_sP_P := -r_P_P_SP: # linke Seite: Variablen-Notation aus SA Job
s_P_P_sP := mr_P_P_SP:
M_plf := M_plf:
I_P_SP := I_P_SP:
I_P_P := I_P_P:
# 3x3-Tr�gheitstensoren aus den gestapelten Vektoren aus der Definitionsdatei bestimmen.
# Ab hier Notation des 3x3-Tr�gheitstensors mit "J" statt "I", aus Studienarbeit von Tim-David Job.
J_SP := vec2symmat(I_P_SP([1, 2, 4, 3, 5, 6], 1), 3): # Reihenfolge in I_P_SP und vec2symmat unterschiedlich definiert
;
J_P_P := vec2symmat(I_P_P([1, 2, 4, 3, 5, 6], 1), 3):
# Lade "robotics_repo_path"-File mit Link zum "imes-robotics-matlab"-Repo
read("../robotics_repo_path"):
# Lade die Funktionen aus dem "imes-robotics-matlab"-Repo
read(sprintf("%s/transformation/maple/proc_eul%s2r", robotics_repo_path, angleConv)):
read(sprintf("%s/transformation/maple/proc_eul%sjac", robotics_repo_path, angleConv)):

# Additional Kinematics
# Berechnung der Rotationsmatrizen

R_0_0_E_t := parse(sprintf("eul%s2r",angleConv))(xE_t(4..6)):
R_0_0_E_s := parse(sprintf("eul%s2r",angleConv))(xE_s(4..6)):
RPYjac_0_t := parse(sprintf("eul%sjac",angleConv))(xE_t(4..6)):
RPYjac_0_s := parse(sprintf("eul%sjac",angleConv))(xE_s(4..6)):
r_0_sP_P := R_0_0_E_s.r_P_sP_P:
if codegen_dynpar = 1 then
  J_P_P := J_SP + M_plf*Multiply(Transpose(vec2skew(r_P_sP_P)),vec2skew(r_P_sP_P)):
else
  J_P_P := J_P_P:
end if:
J_0_P := R_0_0_E_s.J_P_P.Transpose(R_0_0_E_s):

r_0_sP_P := R_0_0_E_t.r_P_sP_P:
rD_0_sP_P := diff~(r_0_sP_P,t):
s_0_P_sP := R_0_0_E_t.s_P_P_sP:
sD_0_P_sP := diff~(s_0_P_sP,t):
for i to 3 do
   for j to 6 do
      rD_0_sP_P(i) := subs({xED_t(j)=xED_s(j),xE_t(j)=xE_s(j)},rD_0_sP_P(i)):
      sD_0_P_sP(i) := subs({xED_t(j)=xED_s(j),xE_t(j)=xE_s(j)},sD_0_P_sP(i)):
   end do:
end do:
s_0_P_sP := R_0_0_E_s.s_P_P_sP:
r_0_sP_P := R_0_0_E_s.r_P_sP_P:

# Berechnung der H-Matrix und deren Ableitung nach Abdellatif2007 S.20
# 
RPYjac_E_t := simplify(Multiply(Transpose(R_0_0_E_t),RPYjac_0_t)):
RPYjac_E_s := simplify(Multiply(Transpose(R_0_0_E_s),RPYjac_0_s)):
w_E_0_E_s := RPYjac_E_s.xED_s(4..6,1):
w_0_0_E_s := RPYjac_0_s.xED_s(4..6,1):
w_E_0_E_t := RPYjac_E_t.xED_t(4..6,1):
dRPYjac_E_t := diff~(RPYjac_E_t,t):
dRPYjac_0_t := diff~(RPYjac_0_t,t):
dRPYjac_E_s := Copy(dRPYjac_E_t):
dRPYjac_0_s := Copy(dRPYjac_0_t):
# Substituiere die zeitabh�ngigen Koordinaten in der H-Matrix mit zeitunabh�ngigen Koordinaten 
for i to 3 do
  for j to 3 do
    for k from 4 to 6 do
      dRPYjac_E_s(i,j) := subs({xEDD_t(k)=xEDD_s(k),xED_t(k)=xED_s(k),xE_t(k)=xE_s(k)},dRPYjac_E_s(i,j)):
      dRPYjac_0_s(i,j) := subs({xEDD_t(k)=xEDD_s(k),xED_t(k)=xED_s(k),xE_t(k)=xE_s(k)},dRPYjac_0_s(i,j)):
    end do:
  end do:
end do:

wD_E_0_E_s := simplify(dRPYjac_E_s.xED_s(4..6,1)+RPYjac_E_s.xEDD_s(4..6,1)):
wD_0_0_E_s := simplify(dRPYjac_0_s.xED_s(4..6,1)+RPYjac_0_s.xEDD_s(4..6,1)):

# Abdellatif2007 S.20 (2.19)

H := <IdentityMatrix(3,3),ZeroMatrix(3);
      ZeroMatrix(3),RPYjac_0_s>:
Hinv := MatrixInverse(H):
dH := <ZeroMatrix(3),ZeroMatrix(3);
      ZeroMatrix(3),dRPYjac_0_s>:

# Gravitational-Vector gE of the platform
# Berechnung des Gravitationsvektors
# Abdellatif2007 S.34 (3.20)
xAll := <x;y;z;an;bn;cn>:
if codegen_dynpar = 1 then
  gE_z := Matrix(xE_s(1..3,1)) - r_0_sP_P:
else
  gE_z := M_plf*Matrix(xE_s(1..3,1)) + s_0_P_sP:
end if:
for j to 3 do
  for i from 1 to 6 do
    gE_z(j) := subs(xE_s(i)=xAll(i),gE_z(j)):
  end do:
end do:
dgEdz_final := Matrix(6,1):
dgEdz := Matrix(3,1):

for j to 6 do
  for i to 3 do
     dgEdz(i) := diff(gE_z(i),xAll(j)):
  end do:
  if codegen_dynpar = 1 then
    dgEdz_final(j) := M_plf*Transpose(g_world(..,1)).dgEdz(..,1):
  else 
    dgEdz_final(j) := Transpose(g_world(..,1)).dgEdz(..,1):
  end if:
end do:
for j to 6 do
  for i from 1 to 6 do
    dgEdz_final(j) := subs(xAll(i)=xE_s(i),dgEdz_final(j)):
  end do:
end do:
gE := simplify(Transpose(Hinv).dgEdz_final):

if codeexport_grav then
  MatlabExport(gE, sprintf("../codeexport/%s/tmp/gravload_platform_eul%s_matlab.m", robot_name, angleConv), codegen_opt):
end if:

# Mass-Matrix ME of the platform
# Berechnung Massenmatrix f�r die EE-Plattform und die Winkelgeschwindigkeit
# Abdellatif2007 S.39 (3.30)

if codegen_dynpar = 1 then
  ME := simplify(<M_plf*Matrix(3,shape=identity),M_plf*vec2skew(r_0_sP_P);
         M_plf*Transpose(vec2skew(r_0_sP_P)),J_0_P>):
else 
  ME := simplify(<M_plf*Matrix(3,shape=identity),vec2skew(-s_0_P_sP);
         Transpose(vec2skew(-s_0_P_sP)),J_0_P>):
end if:

if codeexport_inertia then
  MatlabExport(ME, sprintf("../codeexport/%s/tmp/inertia_platform_eul%s_matlab.m", robot_name, angleConv), codegen_opt):
end if:

# Coriolis-Vector cE of the platform
# Coriolis-Matrix
# Abdellatif2007 S.39 (3.31)

if codegen_dynpar = 1 then
  cE := simplify(<ZeroMatrix(3,3),M_plf*vec2skew(rD_0_sP_P);
         ZeroMatrix(3,3),Multiply(vec2skew(w_0_0_E_s),J_0_P)>):
else
  cE := simplify(<ZeroMatrix(3,3),vec2skew(-sD_0_P_sP);
         ZeroMatrix(3,3),Multiply(vec2skew(w_0_0_E_s),J_0_P)>):
end if:

if codeexport_corvec then
  # Der Term ist nicht der Coriolis-Vektor, sondern eine Matrix, die dazu f�hrt. Daher kein Export.
  # MatlabExport(cE, sprintf("../codeexport/%s/tmp/coriolisvec_platform_eul%s_matlab.m", robot_name, angleConv), codegen_opt):
end if:

# Torque at the platform
# Inverse Dynamik der Plattform
# Abdellatif2007 S.39 (3.29)

MME := ME.H:
cvecE := Multiply(ME.dH+cE.H,xED_s):
tauE := Multiply(MME,xEDD_s) + cvecE - gE:
# Maple-Export
save MME, cvecE , gE, tauE, H, dH, sprintf("../codeexport/%s/tmp/floatb_platform_dynamic_maple.m", robot_name):
# Matlab Code Export
if codeexport_invdyn then
  MatlabExport(tauE, sprintf("../codeexport/%s/tmp/invdyn_platform_eul%s_matlab.m", robot_name, angleConv), codegen_opt):
end if:

