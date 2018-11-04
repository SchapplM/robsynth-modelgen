
# Inverse Dynamics for Robot-Base
# Einleitung
# Berechnung der inversen Dynamik der Roboter-Plattform
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# para -> Berechnung für eine parallelen Roboter
# rotmat -> Kinematik wird mit Rotationsmatrizen berechnet
# dynamics -> Berechnung der Dynamik
# Autor
# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# Sources
# [Abdellatif2007] Modellierung, Identifikation und robuste Regelung von Robotern mit parallelkinematischen Strukturen
# Initialization
#interface(warnlevel=0): # Unterdrücke die folgende Warnung.
restart: # Gibt eine Warnung, wenn über Terminal-Maple mit read gestartet wird.
#interface(warnlevel=3):
with(LinearAlgebra):
with(codegen):
with(CodeGeneration):
with(StringTools):
with(VectorCalculus):
# Einstellungen für Code-Export: Optimierungsgrad (2=höchster) und Aktivierung jedes Terms.
codegen_dynpar := 1:
codegen_opt := 2:
codeexport_invdyn := false:
codeexport_grav := false: 
codeexport_corvec := false:
codeexport_inertia := false:
read "../helper/proc_MatlabExport":
read "../helper/proc_vec2skew":
read "../robot_codegen_definitions/robot_env_par":
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", leg_name):
# Definitionen für parallelen Roboter laden
read "../robot_codegen_definitions/robot_env_par":
read sprintf("../codeexport/%s/tmp/para_definitions", robot_name):
r_P_sP_P := -r_P_sP:
s_P_P_sP := s_P_sP:
# Ergebnisse der Kinematik für parallen Roboter laden
read "../robot_codegen_definitions/robot_env_par":
read sprintf("../codeexport/%s/tmp/kinematics_%s_platform_maple.m", robot_name, base_method_name):
read "../robot_codegen_definitions/robot_env_par":
# Lade "robotics_repo_path"-File mit Link zum "imes-robotics-matlab"-Repo
#read("robotics_repo_path"):
robotics_repo_path := "C:/Users/Tim-David/Documents/Studienarbeit/Repos/imes-robotics-matlab":
# Lade die Funktionen aus dem "imes-robotics-matlab"-Repo
read(sprintf("%s/transformation/maple/proc_eul%s2r", robotics_repo_path, angleConvLeg)):
read(sprintf("%s/transformation/maple/proc_eul%sjac", robotics_repo_path, angleConvLeg)):
# Additional Kinematics
# Berechnung der Rotationsmatrizen
R_0_0_E_t := parse(sprintf("eul%s2r",angleConvLeg))(xE_t(4..6)):
R_0_0_E_s := parse(sprintf("eul%s2r",angleConvLeg))(xE_s(4..6)):
RPYjac_0_t := parse(sprintf("eul%sjac",angleConvLeg))(xE_t(4..6)):
RPYjac_0_s := parse(sprintf("eul%sjac",angleConvLeg))(xE_s(4..6)):
r_0_sP_P := R_0_0_E_s.r_P_sP_P:
if codegen_dynpar = 1 then
  J_P_P := J_SP + mE*Multiply(Transpose(vec2skew(r_P_sP_P)),vec2skew(r_P_sP_P)):
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
RPYjac_E_t := combine(Multiply(Transpose(R_0_0_E_t),RPYjac_0_t)):
RPYjac_E_s := combine(Multiply(Transpose(R_0_0_E_s),RPYjac_0_s)):
w_E_0_E_s := RPYjac_E_s.xED_s(4..6,1):
w_0_0_E_s := RPYjac_0_s.xED_s(4..6,1):
w_E_0_E_t := RPYjac_E_t.xED_t(4..6,1):
dRPYjac_E_t := diff~(RPYjac_E_t,t):
dRPYjac_0_t := diff~(RPYjac_0_t,t):
dRPYjac_E_s := Copy(dRPYjac_E_t):
dRPYjac_0_s := Copy(dRPYjac_0_t):
# Substituiere die zeitabhängigen Koordinaten in der H-Matrix mit zeitunabhängigen Koordinaten 
for i to 3 do
	for j to 3 do
		for k from 4 to 6 do
			dRPYjac_E_s(i,j) := subs({xEDD_t(k)=xEDD_s(k),xED_t(k)=xED_s(k),xE_t(k)=xE_s(k)},dRPYjac_E_s(i,j)):
			dRPYjac_0_s(i,j) := subs({xEDD_t(k)=xEDD_s(k),xED_t(k)=xED_s(k),xE_t(k)=xE_s(k)},dRPYjac_0_s(i,j)):
		end do:
	end do:
end do:

wD_E_0_E_s := dRPYjac_E_s.xED_s(4..6,1)+RPYjac_E_s.xEDD_s(4..6,1):
wD_0_0_E_s := dRPYjac_0_s.xED_s(4..6,1)+RPYjac_0_s.xEDD_s(4..6,1):
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
  gE_z := xE_s(1..3,1) - r_0_sP_P:
else
  gE_z := mE*xE_s(1..3,1) + s_0_P_sP:
end if:
for j to 3 do
  for i from 1 to 6 do
    gE_z(j) := subs(x_all[i]=xAll(i),gE_z(j)):
  end do:
end do:
dgEdz_final := Matrix(6,1):
dgEdz := Matrix(3,1):
gvec := Matrix(3,1,[g1,g2,g3]):
for j to 6 do
  for i to 3 do
     dgEdz(i) := diff(gE_z(i),xAll(j)):
  end do:
  if codegen_dynpar = 1 then
    dgEdz_final(j) := mE*Transpose(gvec(..,1)).dgEdz(..,1):
  else 
    dgEdz_final(j) := Transpose(gvec(..,1)).dgEdz(..,1):
  end if:
end do:
for j to 6 do
  for i from 1 to 6 do
    dgEdz_final(j) := subs(xAll(i)=x_all[i],dgEdz_final(j)):
  end do:
end do:
gE := Transpose(Hinv).dgEdz_final:
if codeexport_grav then
  MatlabExport(gE, sprintf("../codeexport/%s/tmp/gravload_platform_matlab.m", robot_name), codegen_opt):
end if:
# Mass-Matrix ME of the platform
# Berechnung Massenmatrix für die EE-Plattform und die Winkelgeschwindigkeit
# Abdellatif2007 S.39 (3.30)
if codegen_dynpar = 1 then
  ME := <mE*Matrix(3,shape=identity),mE*vec2skew(r_0_sP_P);
         mE*Transpose(vec2skew(r_0_sP_P)),J_0_P>:
  la := codegen_dynpar;
else 
  ME := <mE*Matrix(3,shape=identity),vec2skew(-s_0_P_sP);
         Transpose(vec2skew(-s_0_P_sP)),J_0_P>:
  la := codegen_dynpar;
end if:
if codeexport_inertia then
  MatlabExport(ME, sprintf("../codeexport/%s/tmp/inertia_platform_matlab.m", robot_name), codegen_opt):
end if:
# Coriolis-Vector cE of the platform
# Coriolis-Matrix
# Abdellatif2007 S.39 (3.31)
if codegen_dynpar = 1 then
  cE := <ZeroMatrix(3,3),mE*vec2skew(rD_0_sP_P);
         ZeroMatrix(3,3),Multiply(vec2skew(w_0_0_E_s),J_0_P)>:
else
  cE := <ZeroMatrix(3,3),vec2skew(-sD_0_P_sP);
         ZeroMatrix(3,3),Multiply(vec2skew(w_0_0_E_s),J_0_P)>:
end if:
if codeexport_corvec then
  MatlabExport(cE, sprintf("../codeexport/%s/tmp/coriolisvec_platform_matlab.m", robot_name), codegen_opt):
end if:
# Torque at the platform
# Inverse Dynamik der Plattform
# Abdellatif2007 S.39 (3.29)
MME := ME.H:
cvecE := Multiply(ME.dH+cE.H,xED_s):
tauE := Multiply(ME.H,xEDD_s) + Multiply(ME.dH+cE.H,xED_s) - gE:# - Multiply(Transpose(Hinv),dT):
# Code Export
if codeexport_invdyn then
  MatlabExport(tauE, sprintf("../codeexport/%s/tmp/invdyn_floatb_%s_platform_matlab.m", robot_name, base_method_name), codegen_opt):
end if:
# Maple-Export
save MME, cvecE , gE, tauE, H, dH, sprintf("../codeexport/%s/tmp/floatb_platform_dynamic_maple.m", robot_name):

