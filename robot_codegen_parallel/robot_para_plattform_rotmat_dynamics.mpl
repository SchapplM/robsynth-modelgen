
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
# [GautierKhalil1990] Direct Calculation of Minimum Set of Inertial Parameters of Serial Robots
# [KhalilDombre2002] Modeling, Identification and Control of Robots
# [Ortmaier2014] Vorlesungsskript Robotik I
# Initialization
#interface(warnlevel=0): # Unterdrücke die folgende Warnung.
restart: # Gibt eine Warnung, wenn über Terminal-Maple mit read gestartet wird.
#interface(warnlevel=3):
with(LinearAlgebra):
with(ArrayTools):
with(codegen):
with(CodeGeneration):
with(StringTools):
with(VectorCalculus):
# Einstellungen für Code-Export: Optimierungsgrad (2=höchster) und Aktivierung jedes Terms.
codegen_opt := 2:
codeexport_invdyn := true:
codeexport_grav := true: 
codeexport_corvec := true:
codeexport_inertia := true:
read "../helper/proc_convert_s_t":
read "../helper/proc_convert_t_s": 
read "../helper/proc_MatlabExport":
read "../helper/proc_index_symmat2vector":
read "../helper/proc_symmat2vector":
read "../helper/proc_skew2vec":
read "../helper/proc_vec2skew":
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
# Ergebnisse der Kinematik für parallen Roboter laden
read sprintf("../codeexport/%s/tmp/kinematics_%s_platform_maple.m", robot_name, base_method_name):
# Additional Definitions
# Parameter definieren, wenn nicht vorher schon geschehen
if not assigned(JE) then
   JE := Matrix(3,3,[XX,XY,XZ,YX,YY,YZ,ZX,ZY,ZZ]):
end if:
if not assigned(xE_s) then
   xE_s:=<x_all[1];x_all[2];x_all[3];x_all[4];x_all[5];x_all[6]>:
end if:

# Additional Kinematics
# Berechnung der Winkelgeschwindigkeiten
R_0_0_E := rotx(xE_t(4)).roty(xE_t(5)).rotz(xE_t(6)):
w_E_0_E_t := Multiply(Transpose(R_0_0_E),diff~(R_0_0_E,t)):
w_E_0_E_t := combine(w_E_0_E_t):
w_E_0_E_t := skew2vec(w_E_0_E_t):
# Substituiere die zeitabhängigen Geschwindigkeiten im Winkelgeschwindigkeitsvektor mit zeitunabhängigen Geschwindigkeiten 
w_E_0_E_s := copy(w_E_0_E_t):
for i to 3 do
    w_E_0_E_s(i,1) := subs({diff(xE_t(4),t)=xED_s(4),diff(xE_t(5),t)=xED_s(5),diff(xE_t(6),t)=xED_s(6)},w_E_0_E_t(i,1)):
end do:
# Erzeuge die Jacobi-Matrix: EE-Geschwindigkeiten -> Winkelgeschwindigkeitsvektor w_E_0_E_t
w_Jacobi := Matrix(3):
# i steht für die Zeile der jacobi-matrix
for i to 3 do
	# j steht für die Spalte 
	for j to 3 do
		if xED_s(j+3) = 0 then
			w_Jacobi(i,j) := 0:
		else
			w_Jacobi(i,j) := diff(w_E_0_E_s(i),xED_s(j+3)):
		end if:
	end do:
end do:

# Alternative Berechnung für die Jacobi-Matrix. Allerdings wird keine Rücksicht auf Null-Einträge genommen.
#w_Jacobi := Jacobian(convert(w_E_0_E_s,list),convert(xED_dummy(4..6),list)):
# Berechnung der H-Matrix und deren Ableitung nach Abdellatif S.20 aus "Modellierung, Identifikation und robuste Regelung von Robotern mit parallelkinematischen Strukturen"

# Berechnung des translatorischen Teils der H-Matrix
H_trans := Matrix(3,3):
# i steht für den translatorischen Freiheitsgrad (x,y,z)
for i to 3 do
	if not(xED_s(i) = 0) then
		H_trans(i,i) := 1:
	end if:
end do:
H := <H_trans,ZeroMatrix(3);
      ZeroMatrix(3),w_Jacobi>:
dH := diff~(H,t):
# Substituiere die zeitabhängigen Koordinaten in der Winkelgeschwindigkeits-Jacobi-Matrix mit zeitunabhängigen Koordinaten 
for i to 3 do
  for j to 3 do
    w_Jacobi(i,j) := subs({xE_t(4)=xE_s(4),xE_t(5)=xE_s(5),xE_t(6)=xE_s(6)},w_Jacobi(i,j)):
  end do:
end do:
wD_E_0_E_t := diff~(w_E_0_E_t,t):
# Substitiuere zeitabhängige EE-Koordinaten/-Geschwindigkeiten/-Beschleunigungen in der Ableitung der H-Matrix mit zeitunabhängigen
for i to 6 do
  for k to 6 do
    for j to 6 do
      dH(i,k) := subs({xEDD_t(j)=xEDD_s(j),xED_t(j)=xED_s(j),xE_t(j)=xE_s(j)},dH(i,k)):
    end do:
  end do:
end do:
# Substituiere zeitabhängige EE-Koordinaten/-Geschwindigkeiten/-Beschleunigungen in der Ableitung der H-Matrix mit zeitunabhängigen
wD_E_0_E_s := copy(wD_E_0_E_t):
for i to 3 do
  for j to 6 do
    wD_E_0_E_s(i) := subs({xEDD_t(j)=xEDD_s(j),xED_t(j)=xED_s(j),xE_t(j)=xE_s(j)},wD_E_0_E_t(i)):
  end do:
end do:
# 
#Hinv := MatrixInverse(H);
# Gravitational-Vector gE of the platform
# Berechnung des Gravitationsvektors
gE := <mE*<g1;g2;g3>;ZeroMatrix(3,1)>: #Multiply mit Transpose(Hinv)
;
if codeexport_grav then
  MatlabExport(gE, sprintf("../codeexport/%s/tmp/gravload_floatb_%s_platform_matlab.m", robot_name, base_method_name), codegen_opt):
end if:
# Mass-Matrix ME of the platform
# Berechnung Massenmatrix für die EE-Plattform und die Winkelgeschwindigkeit
ME := <mE*Matrix(3,shape=identity),ZeroMatrix(3);
       ZeroMatrix(3),JE>:
if codeexport_inertia then
  MatlabExport(ME, sprintf("../codeexport/%s/tmp/inertia_floatb_%s_platform_matlab.m", robot_name, base_method_name), codegen_opt):
end if:
# Coriolis-Vector cE of the platform
# Coriolis-Vector
cE := <ZeroMatrix(3,1);vec2skew(w_E_0_E_s).JE.w_E_0_E_s>:
if codeexport_corvec then
  MatlabExport(ME, sprintf("../codeexport/%s/tmp/coriolisvec_floatb_%s_platform_matlab.m", robot_name, base_method_name), codegen_opt):
end if:
# Torque at the platform
# Inverse Dynamik der Plattform
tauE := Multiply(ME,<xEDD_s(1..3,1);wD_E_0_E_s>) + cE - gE:
# Code Export
if codeexport_invdyn then
  MatlabExport(tauE, sprintf("../codeexport/%s/tmp/invdyn_floatb_%s_platform_matlab.m", robot_name, base_method_name), codegen_opt):
end if:
# Maple-Export
save tauE, H, dH, sprintf("../codeexport/%s/tmp/floatb_%s_platform_dynamic_maple.m", robot_name, base_method_name):

