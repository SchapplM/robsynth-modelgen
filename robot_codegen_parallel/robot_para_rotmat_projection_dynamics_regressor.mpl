
# Berechnung und Projektion der Dynamikgleichungen in Regressorform
# Einleitung
# Berechnung und Projektion der Dynamikgleichungen in Regressorform
# 
# Dateiname:
# robot -> Berechnung f�r allgemeinen Roboter
# para -> Berechnung f�r eine parallelen Roboter
# rotmat -> Kinematik wird mit Rotationsmatrizen berechnet
# projection -> Die Dynamikgleichungen werden auf EE-Koordinaten projiziert
# kinematics -> Berechnung der Kinematik
# regressor -> Regressorform (parameterlinear)
# Autor
# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# Sources
# [GautierKhalil1990] Direct Calculation of Minimum Set of Inertial Parameters of Serial Robots
# [KhalilDombre2002] Modeling, Identification and Control of Robots
# [Ortmaier2014] Vorlesungsskript Robotik I
# Initialization
interface(warnlevel=0): # Unterdr�cke die folgende Warnung.
restart: # Gibt eine Warnung, wenn �ber Terminal-Maple mit read gestartet wird.
interface(warnlevel=3):
interface(rtablesize=100):
with(LinearAlgebra):
with(codegen):
with(CodeGeneration):
with(StringTools):
# Einstellungen f�r Code-Export: Optimierungsgrad (2=h�chster) und Aktivierung jedes Terms.
#codegen_act := true: # noch nicht implementiert
codegen_debug := false:
codegen_opt := 1:
codeexport_invdyn := true:
read "../helper/proc_MatlabExport": 
read "../robot_codegen_definitions/robot_env_par":
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", leg_name):
read "../robot_codegen_definitions/robot_env_par":
# Kennung des Parametersatzes, f�r den die Dynamikfunktionen erstellt werden sollen. Muss im Repo und in der mpl-Datei auf 1 gelassen werden, da die folgende Zeile mit einem Skript verarbeitet wird.
codegen_dynpar := 2:
# Link-Index, f�r den die Jacobi-Matrix aufgestellt wird. Hier wird angenommen, dass der Endeffektor das letzte Segment (=Link) ist. Die Jacobi-Matrix kann hier aber f�r beliebige Segmente aufgestellt werden. (0=Basis)
LIJAC:=NL-1:
regressor_modus := "regressor_minpar":
# Ergebnisse der Plattform-Dynamik in Regressorform laden
read sprintf("../codeexport/%s/tmp/floatb_%s_platform_dynamic_maple.m", robot_name, base_method_name):
# Ergebnisse der Dynamik der Gelenkkette in Regressorform laden
read sprintf("../codeexport/%s/tmp/invdyn_%s_%s_maple.m", leg_name, "fixb", regressor_modus):
read "../robot_codegen_definitions/robot_env_par":
# Ergebnisse des Minimalparametervektors der Gelenkkette laden
read sprintf("../codeexport/%s/tmp/minimal_parameter_vector_fixb_maple", leg_name):
read "../robot_codegen_definitions/robot_env_par":
# Ergebnisse der zus�tzlichen Definitionen f�r parallele Roboter laden
read sprintf("../codeexport/%s/tmp/para_definitions", robot_name):
# Ergebnisse der Kinematik f�r parallen Roboter laden
read sprintf("../codeexport/%s/tmp/kinematics_%s_platform_maple.m", robot_name, base_method_name):
printf("Generiere Parameterlineare Form der Dynamik f�r PKM %s mit Parametersatz %d\n", robot_name, codegen_dynpar, base_method_name):
# Berechne Dynamik-Matrizen f�r alle Beine
# Alle Basisgeschwindigkeiten und -winkel aus Berechnung der seriellen Kette zu null setzen.
omegaxs_base := 0:
omegays_base := 0:
omegazs_base := 0:
alphaxs_base := 0:
betays_base := 0:
gammazs_base := 0:
vxs_base := 0:
vys_base := 0:
vzs_base := 0:
# Physikalische Parameter der Koppelgelenke zu Null setzen.
NQ := NQ - (NQJ-NQJ_parallel):
for i from NQJ_parallel+1 to NQJ do
	XXC||i := 0:
	XYC||i := 0:
	XZC||i := 0:
	YYC||i := 0:
	YZC||i := 0:
	ZZC||i := 0:
	XX||i := 0:
	XY||i := 0:
	XZ||i := 0:
	YY||i := 0:
	YZ||i := 0:
	ZZ||i := 0:
	SX||i := 0:
	SY||i := 0:
	SZ||i := 0:
	MX||i := 0:
	MY||i := 0:
	MZ||i := 0:
	M||i := 0:
end do:
# Ergebnisse der Kinematik f�r parallelen Roboter laden
read sprintf("../codeexport/%s/tmp/kinematics_%s_platform_maple.m", robot_name, base_method_name):
Paramvec2 := Paramvec2:
for j to RowDimension(Paramvec2) do
	if Paramvec2(j) = M(NQJ_parallel+1,1) then
		Paramvec2(j) := 0;
		paramVecP := paramVecP_M;
	end if:
end do:



Paramvec2 := remove(has,Paramvec2,0):
counter := 0:
for i to RowDimension(Paramvec2) do
	if not(Paramvec2(i,1) = NULL) then
		if counter = 0 then
			ParamvecNew := Paramvec2(i,1):
			counter := 1:
		else
			ParamvecNew := <ParamvecNew;Paramvec2(i,1)>:
		end if:
	end if:
end do:

# Dupliziere alle berechneten Matrizen. i steht f�r den Index des jeweiligen Beines
g1 := gtmp1:
g2 := gtmp2:
g3 := gtmp3:
unassign('g1','g2','g3'):
g := <g1;g2;g3>:
Rmat := Transpose(parse(sprintf("eul%s2r",angleConvLeg))(frame_A_i(1..3,1))):
gtmp1 := (Rmat.g)(1):
gtmp2 := (Rmat.g)(2):
gtmp3 := (Rmat.g)(3):
for i to N_LEGS do
  tau_regressor_s||i := Copy(tau_regressor_s(7..NQ,..)):
end do:
COLUMNreg := ColumnDimension(tau_regressor_s):
# Substituiere in jeder Matrix den Winkel Alpha (Verdrehung in der Basis) und die Gelenkkoordinaten und -geschwindigkeiten
for k from 1 by 1 to N_LEGS do
  	for i to NQJ_parallel do
  		for j to COLUMNreg do
  			for p from NQJ_parallel+1 to NQJ do
  				tau_regressor_s||k(i,j) := subs({qJDD||p||s=0,qJD||p||s=0,qJ||p||s=0},tau_regressor_s||k(i,j)):
  			end do:
  			for l to 3 do
  	 			tau_regressor_s||k(i,j) := subs({frame_A_i(l,1)=frame_A_i(l,k)},tau_regressor_s||k(i,j)):
  			end do:
  			for m to NQJ_parallel do
  				n := (m + (k-1)*NQJ_parallel):
     			tau_regressor_s||k(i,j) := subs({qJDD||m||s=qJDD||n||s,qJD||m||s=qJD||n||s,qJ||m||s=qJ||n||s},tau_regressor_s||k(i,j)):
     		end do:
  		end do:
  	end do:
end do:

# Berechnung, Projektion und Addition der Dynamikgleichungen
# Berechnung der Kr�fte/Momente an den Gelenken der jeweiligen Beine und Projektion auf EE-Plattform
A_E := A_E:
U_i := U_i:
JBinv_i := JBinv_i:
for i to N_LEGS do
   A_||i:= simplify(Transpose(U_i(..,..,i)).Transpose(JBinv_i(..,..,i))).simplify(tau_regressor_s||i):
end do:
# Aufsummieren aller Kr�fte, projiziert auf EE-Plattform
Tmp := 0:
for i to N_LEGS do
  Tmp := Tmp + A_||i:
end do:
A_j := Tmp:
# Addiere Inverse Dynamik der Plattform

ROWS := RowDimension(ParamvecNew):
A := pivotMat.<A_j(1..6,1..ROWS)|A_E>:
paramMin := <ParamvecNew;paramVecP>:
NotNullEntries := 0:
for i to RowDimension(paramMin) do
	if not(paramMin(i,1) = 0 or Equal(Column(A,i), ZeroVector[column](RowDimension(A)))) then
		NotNullEntries := NotNullEntries + 1:
	end if:
end do:
NotNullEntries:
paramMinRed := Matrix(NotNullEntries,1):
ARed := Matrix(RowDimension(A),NotNullEntries):
j := 1:
for i to RowDimension(paramMin) do
	if not(paramMin(i,1) = 0 or Equal(Column(A,i), ZeroVector[column](RowDimension(A)))) then
		paramMinRed(j,1) := paramMin(i,1):
		for k to RowDimension(A) do
			ARed(k,j) := A(k,i):
		end do:
		j := j + 1:
	end if:
end do:

# Replace Joint Velocities
# Substituiere die Gelenkgeschwindigkeiten �ber H-, Ui- und JBi-Matrix mit EE-Geschwindikeiten
Tmp := 0:
Tmp2 := 0:
for i to N_LEGS do
  Tmp := Multiply(H,xED_s):
  Tmp := Multiply(U_i(..,..,i),Tmp):
  z||i := simplify(Multiply(JBinv_i(..,..,i),Tmp)):
  Tmp2 := JBinv_i(..,..,i).(Multiply(U_i(..,..,i),H).xEDD_s+Multiply(U_i(..,..,i),dH).xED_s+Multiply(UD_i(..,..,i),H).xED_s):
  A||i := simplify(Multiply(JBinv_i(..,..,i),JBD_i(..,..,i))):
  B||i := Multiply(A||i,Matrix(qJD_i_s(..,i))):
  C||i := Tmp2 - B||i:
end do:
RowParamMin := RowDimension(paramMinRed):
for i to NX do
  for k to RowParamMin do
    for j to N_LEGS do
      for l to NQJ_parallel do
        ARed(i,k) := subs({qJDD_i_s(l,j)=C||j(l)},ARed(i,k)):
      end do:
    end do:
    for j to N_LEGS do
      for l to NQJ_parallel do
        ARed(i,k) := subs({qJD_i_s(l,j)=z||j(l)},ARed(i,k)):
      end do:
    end do:
  end do:
end do:
tauGes := ARed:#.paramMinRed:
if RowDimension(Jinv) < 4 then
  J:=MatrixInverse(Jinv):
  J:=simplify(J):
  tau:=Transpose(J).tauGes:
else
  tau := tauGes:
end if:
# Export
#tau := tauGes:
# Matlab Export: Floating base
# Berechnung der Basis-Belastung ist f�r manche Basis-Darstellungen falsch (siehe oben unter Gravitationslast).
if codeexport_invdyn then
  MatlabExport(tau, sprintf("../codeexport/%s/tmp/invdyn_para_reg_matlab.m", robot_name), codegen_opt):
end if:
MatlabExport(paramMinRed, sprintf("../codeexport/%s/tmp/minimal_parameter_parrob_matlab.m", robot_name), codegen_opt):
MatlabExport(RowParamMin, sprintf("../codeexport/%s/tmp/RowMinPar_parallel.m", robot_name), 2);

