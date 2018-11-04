
# Berechnung und Projektion der Dynamikgleichungen
# Einleitung
# Berechnung und Projektion der Dynamikgleichungen
# 
# Dateiname:
# robot -> Berechnung f�r allgemeinen Roboter
# para -> Berechnung f�r eine parallelen Roboter
# rotmat -> Kinematik wird mit Rotationsmatrizen berechnet
# projection -> Die Dynamikgleichungen werden auf EE-Koordinaten projiziert
# kinematics -> Berechnung der Kinematik
# Autor
# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# Sources
# [Abdellatif2007] Modellierung, Identifikation und robuste Regelung von Robotern mit parallelkinematischen Strukturen
# Initialization
#interface(warnlevel=0): # Unterdr�cke die folgende Warnung.
restart: # Gibt eine Warnung, wenn �ber Terminal-Maple mit read gestartet wird.
#interface(warnlevel=3):
with(LinearAlgebra):
with(codegen):
with(CodeGeneration):
with(StringTools):
# Einstellungen f�r Code-Export: Optimierungsgrad (2=h�chster).
#codegen_act := true: # noch nicht implementiert
codegen_debug := false:
codegen_opt := 2:
codeexport_invdyn := true:
read "../helper/proc_MatlabExport":
read "../robot_codegen_definitions/robot_env_par":
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", leg_name):
# Kennung des Parametersatzes, f�r den die Dynamikfunktionen erstellt werden sollen. Muss im Repo und in der mpl-Datei auf 1 gelassen werden, da die folgende Zeile mit einem Skript verarbeitet wird.
codegen_dynpar := 1:
# Link-Index, f�r den die Jacobi-Matrix aufgestellt wird. Hier wird angenommen, dass der Endeffektor das letzte Segment (=Link) ist. Die Jacobi-Matrix kann hier aber f�r beliebige Segmente aufgestellt werden. (0=Basis)
LIJAC:=NL-1:
# Ergebnisse der zus�tzlichen Definitionen f�r parallele Roboter laden
read "../robot_codegen_definitions/robot_env_par":
read sprintf("../codeexport/%s/tmp/para_definitions", robot_name):
# Ergebnisse der Plattform-Dynamik laden
read "../robot_codegen_definitions/robot_env_par":
read sprintf("../codeexport/%s/tmp/floatb_platform_dynamic_maple.m", robot_name):
# Ergebnisse der Dynamik der Gelenkkette laden
#read sprintf("../codeexport/%s/tmp/invdyn_%s_par%d_maple.m", leg_name, base_method_name, codegen_dynpar)
;
# Ergebnisse der Kinematik f�r parallelen Roboter laden
read "../robot_codegen_definitions/robot_env_par":
read sprintf("../codeexport/%s/tmp/kinematics_%s_platform_maple.m", robot_name, base_method_name):
read "../robot_codegen_definitions/robot_env_par":
# Lade "robotics_repo_path"-File mit Link zum "imes-robotics-matlab"-Repo
#read("robotics_repo_path"):
robotics_repo_path := "C:/Users/Tim-David/Documents/Studienarbeit/Repos/imes-robotics-matlab":
# Lade die Funktionen aus dem "imes-robotics-matlab"-Repo
read(sprintf("%s/transformation/maple/proc_eul%s2r", robotics_repo_path, angleConvLeg)):
read(sprintf("%s/transformation/maple/proc_eul%sjac", robotics_repo_path, "zyx")):
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
# Ergebnisse G-Vektor laden. Die Rotation der Basis wird nur in der Jacobi-Matrix der inverse Kinematik ber�cksichtigt. Deshalb muss der Gravitationsvektor ebenfalls an die Rotation angepasst werden.
g1 := gtmp1:
g2 := gtmp2:
g3 := gtmp3:
read sprintf("../codeexport/%s/tmp/gravload_par%d_maple.m", leg_name, codegen_dynpar):
G := Matrix(taug_s(7..NQ,1)):
unassign('g1','g2','g3'):
g := <g1;g2;g3>:
Rmat := Transpose(parse(sprintf("eul%s2r",angleConvLeg))(frame_A_i(1..3,1))):
gtmp1 := (Rmat.g)(1):
gtmp2 := (Rmat.g)(2):
gtmp3 := (Rmat.g)(3):
# Ergebnisse C-Vektor laden
read sprintf("../codeexport/%s/tmp/coriolisvec_par%d_maple.m", leg_name, codegen_dynpar):
Cvec := combine(Matrix(tauCC_s(7..NQ,1))):
# Ergebnisse M-Matrix laden
read sprintf("../codeexport/%s/tmp/inertia_par%d_maple.m", leg_name, codegen_dynpar):
MM := combine(MM_s(7..NQ,7..NQ)):
# Ergebnisse der Kinematik f�r parallen Roboter laden
read sprintf("../codeexport/%s/tmp/kinematics_%s_platform_maple.m", robot_name, base_method_name):
printf("Generiere Dynamik f�r PKM %s mit Parametersatz %d\n", robot_name, codegen_dynpar, base_method_name):
# Berechne Dynamik-Matrizen f�r alle Beine
# Dupliziere alle berechneten Matrizen. i steht f�r den Index des jeweiligen Beines
for i to N_LEGS do
  MM||i := Copy(MM):
  Cvec||i := Copy(Cvec):
  G||i := Copy(G):
end do:
# Substituiere in jeder Matrix den Winkel Alpha (Verdrehung in der Basis) und die Gelenkkoordinaten und -geschwindigkeiten
for k from 1 by 1 to N_LEGS do
  	for i to NQJ_parallel do
  		for l to 3 do
  	 		Cvec||k(i,1):=subs({frame_A_i(l,1)=frame_A_i(l,k)},Cvec||k(i,1)):
  	  		G||k(i,1):=subs({frame_A_i(l,1)=frame_A_i(l,k)},G||k(i,1)):
  		end do:
    		for m to NQJ_parallel do #alpha
      		n := (m + (k-1)*NQJ_parallel):
     		Cvec||k(i,1):=subs({qJD||m||s=qJ||D||n||s,qJ||m||s=qJ||n||s},Cvec||k(i,1)):
      		G||k(i,1):=subs({qJ||m||s=qJ||n||s},G||k(i,1)):
    		end do:
    		for j to NQJ_parallel do
    			for l to 3 do
    	 			MM||k(i,j):=subs({rame_A_i(l,1)=frame_A_i(l,k)},MM||k(i,j)):
    	 		end do:
      		for m to NQJ_parallel do #alpha
        			n := m + (k-1)*NQJ_parallel:
        			MM||k(i,j):=subs({qJ||m||s=qJ||n||s},MM||k(i,j)):
      		end do:
    		end do:
  	end do:
end do:

# Berechnung, Projektion und Addition der Dynamikgleichungen
# Berechnung der Kr�fte/Momente an den Gelenken der jeweiligen Beine und Projektion auf EE-Plattform
# Abdellatif2007 S.38 (3.27)
for i to N_LEGS do
  A||i := Multiply(JBinv_i(..,..,i),JBD_i(..,..,i));
  B||i := Multiply(-MM||i,Multiply(A||i,Multiply(JBinv_i(..,..,i),U_i(..,..,i).H.xED_s)));
  MMs||i := Transpose(U_i(..,..,i)).Transpose(JBinv_i(..,..,i)).MM||i.JBinv_i(..,..,i).U_i(..,..,i).H;
  cvecs||i := Transpose(U_i(..,..,i)).Transpose(JBinv_i(..,..,i)).MM||i.JBinv_i(..,..,i).(U_i(..,..,i).dH+UD_i(..,..,i).H).xED_s+Multiply(Transpose(U_i(..,..,i)),Multiply(Transpose(JBinv_i(..,..,i)),(B||i+Cvec||i)));
  gvecs||i := Multiply(Transpose(U_i(..,..,i)),Multiply(Transpose(JBinv_i(..,..,i)),G||i));
  
  tau||i := Transpose(U_i(..,..,i)).Transpose(JBinv_i(..,..,i)).MM||i.JBinv_i(..,..,i).(U_i(..,..,i).H.xEDD_s+U_i(..,..,i).dH.xED_s+UD_i(..,..,i).H.xED_s) + Multiply(Transpose(U_i(..,..,i)),Multiply(Transpose(JBinv_i(..,..,i)),(B||i+Cvec||i+G||i))):
  taus||i := MMs||i.xEDD_s + cvecs||i + gvecs||i;
end do:
# Abdellatif2007 S.40 (3.33)
# Aufsummieren aller Kr�fte, projiziert auf EE-Plattform
Tmp := 0:
for i to N_LEGS do
  Tmp := Tmp + tau||i:
end do:
# Addiere Inverse Dynamik der Plattform
tauGes := Tmp + tauE:
# Aufsummieren aller Massenmatrizen, projiziert auf EE-Plattform
Tmp := 0:
for i to N_LEGS do
  Tmp := Tmp + MMs||i:
end do:
# Addiere Massenmatrix der Plattform
MMGes := Tmp + MME:
# Aufsummieren aller Coriolisvektoren, projiziert auf EE-Plattform
Tmp := 0:
for i to N_LEGS do
  Tmp := Tmp + cvecs||i:
end do:
# Addiere Coriolisvektor der Plattform
cvecGes := Tmp + cvecE:
# Aufsummieren aller Gravitiationsvektoren, projiziert auf EE-Plattform
Tmp := 0:
for i to N_LEGS do
  Tmp := Tmp + gvecs||i:
end do:
# Addiere Gravitiationsvektor der Plattform
gGes := Tmp - gE:
tauGes := MMGes.xEDD_s + cvecGes + gGes:
# Replace Joint Velocities
# Substituiere die Gelenkgeschwindigkeiten �ber H-, Ui- und JBi-Matrix mit EE-Geschwindikeiten
Tmp := 0:
for i to N_LEGS do
  Tmp := Multiply(H,xED_s):
  Tmp := Multiply(U_i(..,..,i),Tmp):
  z||i := Multiply(JBinv_i(..,..,i),Tmp):
end do:
for i to 6 do
  for j to N_LEGS do
    for l to NQJ_parallel do
      tauGes(i,1) := subs({qJD_i_s(l,j)=z||j(l)},tauGes(i,1)):
      cvecGes(i,1) := subs({qJD_i_s(l,j)=z||j(l)},cvecGes(i,1)):
      gGes(i,1) := subs({qJD_i_s(l,j)=z||j(l)},gGes(i,1)):
      for k to 6 do
        MMGes(i,k) := subs({qJD_i_s(l,j)=z||j(l)},MMGes(i,k)):
      end do:
    end do:
  end do:
end do:

# Export
# W�hle die Eintr�ge aus Dynamikgleichungen, die f�r Freiheitsgrade des Roboters relevant sind.
g1 := 0:
g2 := 0:

tau := pivotMat.tauGes:
MMGes := pivotMat.MMGes.Transpose(pivotMatMas):
cvecGes := pivotMat.cvecGes:
gGes := pivotMat.gGes:
J:=MatrixInverse(Jinv):
tau:=Transpose(J).tau:
#pivotMat.Transpose(pivotMat);
# Matlab Export
if codeexport_invdyn then
  MatlabExport(tau, sprintf("../codeexport/%s/tmp/invdyn_para_par%d_matlab.m", robot_name, codegen_dynpar), codegen_opt);
  #MatlabExport(MMGes, sprintf("../codeexport/%s/tmp/inertia_para_par%d_matlab.m", robot_name, codegen_dynpar), #codegen_opt);
  #MatlabExport(cvecGes, sprintf("../codeexport/%s/tmp/coriolisvec_para_par%d_matlab.m", robot_name, codegen_dynpar), #codegen_opt);
  #MatlabExport(gGes, sprintf("../codeexport/%s/tmp/gravvec_para_par%d_matlab.m", robot_name, codegen_dynpar), codegen_opt)#;
end if:

