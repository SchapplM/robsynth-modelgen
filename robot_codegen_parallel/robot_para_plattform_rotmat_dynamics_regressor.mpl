
# Parameter Regressor Inverse Dynamics for Robot-Base
# Einleitung
# Berechnung der inversen Dynamik der Roboter-Plattform in Regressorform
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# para -> Berechnung für eine parallelen Roboter
# rotmat -> Kinematik wird mit Rotationsmatrizen berechnet
# dynamics -> Berechnung der Dynamik
# regressor -> Regressorform (parameterlinear)
# Autor
# Tim Job (Studienarbeit bei Moritz Schappler), 2018-12
# Moritz Schappler, moritz.schappler@imes.uni-hannover.de
# (C) Institut für Mechatronische Systeme, Universität Hannover
# Sources
# [GautierKhalil1990] Direct Calculation of Minimum Set of Inertial Parameters of Serial Robots
# [KhalilDombre2002] Modeling, Identification and Control of Robots
# [Ortmaier2014] Vorlesungsskript Robotik I
# [Abdel2007] Modellierung, Identifikation und robuste Regelung von Robotern mit parallelkinematischen Strukturen
# Initialization
interface(warnlevel=0): # Unterdrücke die folgende Warnung.
restart: # Gibt eine Warnung, wenn über Terminal-Maple mit read gestartet wird.
interface(warnlevel=3):
with(LinearAlgebra):
#with(ArrayTools):
with(codegen):
with(CodeGeneration):
with(StringTools):
with(VectorCalculus):
# Einstellungen für Code-Export: Optimierungsgrad (2=höchster) und Aktivierung jedes Terms.
codegen_opt := 2:
codeexport_invdyn := false:
codeexport_grav := false: 
codeexport_corvec := false:
codeexport_inertia := false:
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
read "../robot_codegen_definitions/robot_env_par":
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", leg_name):
read "../robot_codegen_definitions/robot_env_par":
# Definitionen für parallelen Roboter laden
read sprintf("../codeexport/%s/tmp/para_definitions", robot_name):
r_P_sP_P := -r_P_sP:
s_P_P_sP := s_P_sP:
# Ergebnisse der Kinematik für parallen Roboter laden
read sprintf("../codeexport/%s/tmp/kinematics_%s_platform_maple.m", robot_name, base_method_name):
# Lade "robotics_repo_path"-File mit Link zum "imes-robotics-matlab"-Repo
read("../robotics_repo_path"):
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
J_P_P := J_P_P:
J_0_P := R_0_0_E_s.J_P_P.Transpose(R_0_0_E_s):
J_0_P_raute := Matrix(6,1,[J_0_P(1,1),J_0_P(1,2),J_0_P(1,3),J_0_P(2,2),J_0_P(2,3),J_0_P(3,3)]):
J_P_P_raute := Matrix(6,1,[J_P_P(1,1),J_P_P(1,2),J_P_P(1,3),J_P_P(2,2),J_P_P(2,3),J_P_P(3,3)]):
r_0_sP_P := R_0_0_E_t.r_P_sP_P:
r_0_P_sP := -r_0_sP_P:
#rD_0_sP_P := diff~(r_0_sP_P,t):
#for i to 3 do
  # for j to 6 do
      #rD_0_sP_P(i) := subs({xED_t(j)=xED_s(j),xE_t(j)=xE_s(j)},rD_0_sP_P(i)):
   #end do:
#end do:
#rD_0_sP_P:
r_0_sP_P := R_0_0_E_s.r_P_sP_P:
r_0_P_sP := -r_0_sP_P:
r_P_P_sP := -r_P_sP_P:
#sE := Matrix(mE*r_0_P_sP):
#sE := Matrix(mE*r_P_P_sP):
sE := s_P_P_sP:
# Berechnung der H-Matrix und deren Ableitung nach Abdellatif S.20 aus "Modellierung, Identifikation und robuste Regelung von Robotern mit parallelkinematischen Strukturen"
RPYjac_E_t := simplify(Multiply(Transpose(R_0_0_E_t),RPYjac_0_t)):
RPYjac_E_s := simplify(Multiply(Transpose(R_0_0_E_s),RPYjac_0_s)):
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
omegaE := <omega1;omega2;omega3>:
xED_dummy := <xE,yE,zE>:
omega0 := Transpose(R_0_0_E_s).omegaE:
<xED_dummy;omegaE>:
JR := Jacobian(convert(omega0,list),convert(<xED_dummy;omegaE>,list)):
JR_T := Transpose(JR):
vel0 := Transpose(R_0_0_E_s).xED_dummy:
JT := Jacobian(convert(vel0,list),convert(<xED_dummy;omegaE>,list)):
JT_T := Transpose(JT):
wD_E_0_E_s := dRPYjac_E_s.xED_s(4..6,1)+RPYjac_E_s.xEDD_s(4..6,1):
wD_0_0_E_s := dRPYjac_0_s.xED_s(4..6,1)+RPYjac_0_s.xEDD_s(4..6,1):
H := <IdentityMatrix(3,3),ZeroMatrix(3);
      ZeroMatrix(3),RPYjac_0_s>:
#MatlabExport(H, "H.m",2);
Hinv := MatrixInverse(H):
dH := <ZeroMatrix(3),ZeroMatrix(3);
      ZeroMatrix(3),dRPYjac_0_s>:
a_E := Matrix(xEDD_s(1..3,1)) - g_world:
a_E := Transpose(R_0_0_E_s).(Matrix(xEDD_s(1..3,1)) - g_world):
w_0_0_E_s_stern := Matrix(3,6,[w_0_0_E_s(1),w_0_0_E_s(2),w_0_0_E_s(3),0,0,0,0,w_0_0_E_s(1),0,w_0_0_E_s(2),w_0_0_E_s(3),0,0,0,w_0_0_E_s(1),0,w_0_0_E_s(2),w_0_0_E_s(3)]):
wD_0_0_E_s_stern := Matrix(3,6,[wD_0_0_E_s(1),wD_0_0_E_s(2),wD_0_0_E_s(3),0,0,0,0,wD_0_0_E_s(1),0,wD_0_0_E_s(2),wD_0_0_E_s(3),0,0,0,wD_0_0_E_s(1),0,wD_0_0_E_s(2),wD_0_0_E_s(3)]):
w_E_0_E_s_stern := Matrix(3,6,[w_E_0_E_s(1),w_E_0_E_s(2),w_E_0_E_s(3),0,0,0,0,w_E_0_E_s(1),0,w_E_0_E_s(2),w_E_0_E_s(3),0,0,0,w_E_0_E_s(1),0,w_E_0_E_s(2),w_E_0_E_s(3)]):
wD_E_0_E_s_stern := Matrix(3,6,[wD_E_0_E_s(1),wD_E_0_E_s(2),wD_E_0_E_s(3),0,0,0,0,wD_E_0_E_s(1),0,wD_E_0_E_s(2),wD_E_0_E_s(3),0,0,0,wD_E_0_E_s(1),0,wD_E_0_E_s(2),wD_E_0_E_s(3)]):
paramVecP_old := <J_0_P_raute;sE;mE>:
paramVecP_old := <J_P_P_raute;sE;mE>:
paramVecP := <J_0_P_raute;sE;mE>:
paramVecP := <J_P_P_raute;sE;mE>:
paramVecP_M := Copy(paramVecP):
tmp := Matrix(10,1):
for i to N_LEGS do
   tmpSt := M(NQJ_parallel+1,1)*Multiply(Transpose(vec2skew(P_i(1..3,i))),vec2skew(P_i(1..3,i))):
   tmpSt_raute := Matrix(6,1,[tmpSt(1,1),tmpSt(1,2),tmpSt(1,3),tmpSt(2,2),tmpSt(2,3),tmpSt(3,3)]):
   tmp := <tmpSt_raute,M(NQJ_parallel+1,1)*P_i(1..3,i),M(NQJ_parallel+1,1)>:
   paramVecP_M := paramVecP_M + tmp:
end do:
A_E := <ZeroMatrix(3,6),vec2skew(wD_0_0_E_s)+Multiply(vec2skew(w_0_0_E_s),vec2skew(w_0_0_E_s)),a_E;
        wD_0_0_E_s_stern + Multiply(vec2skew(w_0_0_E_s),w_0_0_E_s_stern),-vec2skew(a_E),ZeroMatrix(3,1)>:
A_E := <JT_T|JR_T>.<ZeroMatrix(3,6),vec2skew(wD_E_0_E_s)+Multiply(vec2skew(w_E_0_E_s),vec2skew(w_E_0_E_s)),a_E;
        wD_E_0_E_s_stern + Multiply(vec2skew(w_E_0_E_s),w_E_0_E_s_stern),-vec2skew(a_E),ZeroMatrix(3,1)>:
#tau_regressor := A_E.paramVecP:
#MatlabExport(tau_regressor, "taureg.m", 2):
# Mass Matrix
M_regmin := <JT_T|JR_T>.<ZeroMatrix(3,6),vec2skew(wD_E_0_E_s),Transpose(R_0_0_E_s).Matrix(xEDD_s(1..3,1));
             wD_E_0_E_s_stern,-vec2skew(Transpose(R_0_0_E_s).Matrix(xEDD_s(1..3,1))),ZeroMatrix(3,1)>:

MM_regmin := Matrix(6*6, RowDimension(paramVecP)):
i_rr := 0:
for i to 6 do # Zeilenindex der Massenmatrix
  for j to 6 do  # Spaltenindex der Massenmatrix
    i_rr := i_rr + 1: # Gehe zeilenweise durch den unteren linken Teil der Massenmatrix (inkl. Diagonale)
    for k to RowDimension(paramVecP) do # Spaltenindex der Regressormatrix
    	 if not(xEDD_s[j, 1] = 0) then
        MM_regmin[i_rr, k] := diff(M_regmin[i, k], xEDD_s[j, 1]):
      else
        MM_regmin[i_rr, k] := 0:
      end if:
    end do:
  end do:
end do:

# Coriolis Vector
c_regmin := <JT_T|JR_T>.<ZeroMatrix(3,6),Multiply(vec2skew(w_E_0_E_s),vec2skew(w_E_0_E_s)),ZeroMatrix(3,1);
             Multiply(vec2skew(w_E_0_E_s),w_E_0_E_s_stern),ZeroMatrix(3,3),ZeroMatrix(3,1)>:
# Graviational Vector
g_regmin := <JT_T|JR_T>.<ZeroMatrix(3,6),ZeroMatrix(3,3),-Transpose(R_0_0_E_s).g_world;
             ZeroMatrix(3,6),-vec2skew(-Transpose(R_0_0_E_s).g_world),ZeroMatrix(3,1)>:
# Torque at the platform
# Code Export
if codeexport_invdyn then
  MatlabExport(A_E, sprintf("../codeexport/%s/tmp/invdyn_floatb_%s_platform_matlab.m", robot_name, base_method_name), codegen_opt):
  MatlabExport(M_regmin, sprintf("../codeexport/%s/tmp/invdyn_floatb_%s_Mplatform_matlab.m", robot_name, base_method_name), codegen_opt):
  MatlabExport(MM_regmin, sprintf("../codeexport/%s/tmp/invdyn_floatb_%s_MMplatform_matlab.m", robot_name, base_method_name), codegen_opt):
  MatlabExport(c_regmin, sprintf("../codeexport/%s/tmp/invdyn_floatb_%s_cplatform_matlab.m", robot_name, base_method_name), codegen_opt):
  MatlabExport(g_regmin, sprintf("../codeexport/%s/tmp/invdyn_floatb_%s_gplatform_matlab.m", robot_name, base_method_name), codegen_opt):
end if:
# Maple-Export
save paramVecP, paramVecP_M, A_E, M_regmin, c_regmin, g_regmin, H, dH, sprintf("../codeexport/%s/tmp/floatb_%s_platform_dynamic_reg_maple.m", robot_name, base_method_name):

