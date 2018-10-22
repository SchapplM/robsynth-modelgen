
# Kinematik f�r parallelen Roboter
# Einleitung
# Berechnung der Jacobi-Matrix des parallelen Roboters
# 
# Dateiname:
# robot -> Berechnung f�r allgemeinen Roboter
# para -> Berechnung f�r eine parallelen Roboter
# rotmat -> Kinematik wird mit Rotationsmatrizen berechnet
# kinematics -> Berechnung der Kinematik
# Autor
# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# Sources
# [GautierKhalil1990] Direct Calculation of Minimum Set of Inertial Parameters of Serial Robots
# [KhalilDombre2002] Modeling, Identification and Control of Robots
# [Ortmaier2014] Vorlesungsskript Robotik I
# Initialization
#interface(warnlevel=0): # Unterdr�cke die folgende Warnung.
restart: # Gibt eine Warnung, wenn �ber Terminal-Maple mit read gestartet wird.
#interface(warnlevel=3):
with(LinearAlgebra):
#with(ArrayTools):
with(codegen):
with(CodeGeneration):
with(StringTools):
# Einstellungen f�r Code-Export: Optimierungsgrad (2=h�chster) und Aktivierung jedes Terms.
#codegen_act := true: # noch nicht implementiert
codegen_debug := false:
codegen_opt := 2:
codeexport_invdyn := true:
read "../helper/proc_convert_s_t":
read "../helper/proc_convert_t_s": 
read "../helper/proc_MatlabExport":
read "../helper/proc_index_symmat2vector":
read "../helper/proc_symmat2vector":
read "../helper/proc_vec2skew":
read "../helper/proc_skew2vec":
read "../transformation/proc_rotx": 
read "../transformation/proc_roty": 
read "../transformation/proc_rotz": 
read "../transformation/proc_trotx": 
read "../transformation/proc_troty": 
read "../transformation/proc_trotz": 
read "../transformation/proc_rpyjac": 
read "../transformation/proc_transl": 
read "../transformation/proc_trafo_mdh": 
read "../robot_codegen_definitions/robot_env":
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name):
# Definitionen f�r parallel Roboter laden
read sprintf("../codeexport/%s/tmp/para_definitions", robot_name):
# Kennung des Parametersatzes, f�r den die Dynamikfunktionen erstellt werden sollen. Muss im Repo und in der mpl-Datei auf 1 gelassen werden, da die folgende Zeile mit einem Skript verarbeitet wird.
codegen_dynpar := 2:
# Link-Index, f�r den die Jacobi-Matrix aufgestellt wird. Hier wird angenommen, dass der Endeffektor das letzte Segment (=Link) ist. Die Jacobi-Matrix kann hier aber f�r beliebige Segmente aufgestellt werden. (0=Basis)
LIJAC:=NL-1:
# Ergebnisse der analytischen Jacobi-Matrix (Translatorisch)
read sprintf("../codeexport/%s/tmp/jacobia_transl_%d_maple.m", robot_name, LIJAC):
b_transl := b_transl:
px, py, pz := 0, 0, 0:
alphaxs_base, betays_base, gammazs_base := 0, 0, 0: 
# Additional Kinematics
# Definition der Koppelpunkte f�r jedes Bein und der EE-Koordinaten/-Geschwindigkeiten/-Beschleunigungen
tmp := Matrix(3,1,[xP[1],yP[1],zP[1]]):
for j to 3 do:
    if (xE_s(j) = 0) then
      tmp(j,1) := 0:
    end if:
  end do:
for i to N_LEGS do
  P||i := <xP[i];yP[i];zP[i]>;
  for j to 3 do:
    if (xE_s(j) = 0) then
      P||i(j,1) := 0:
    end if:
  end do:
  j := i+1:
  if i <> 1 then
     tmp := <tmp | P||i>:
  end:
end do:
P_i := tmp:

# Jacobi Matrices (JB1/U1) + Derivates
if angleConvLeg = X_Y_Z then
   JB1 := rotx(frame_A_i(1,1)).roty(frame_A_i(2,1)).rotz(frame_A_i(3,1)).b_transl(1..3,1..NQJ_parallel):
elif angleConvLeg = Z_Y_X then
   JB1 := rotz(frame_A_i(1,1)).roty(frame_A_i(2,1)).rotx(frame_A_i(3,1)).b_transl(1..3,1..NQJ_parallel):
end:
#ColJB1 := ColumnDimension(JB1):
#AppendCol := 3 - ColJB1:
#JB1 := <JB1|ZeroMatrix(3,AppendCol)>:
# Berechnung der Jacobi-Matrix JB1inv: Gelenkgeschwindigkeiten -> Koppelpunktgeschwindigkeiten P
#for i to 3 do
#	if JB1(1,i) = 0 and JB1(2,i) = 0 and JB1(3,i) = 0
#	and JB1(i,1) = 0 and JB1(i,2) = 0 and JB1(i,3) = 0 then
#		JB1 := JB1(..,1..i-1):
#    	end if:
#end do:
# Berechnung der Jacobi-Matrix JBE: Koppelpunktgeschwindigkeiten P -> Gelenkgeschwindigkeiten
JB1inv := MatrixInverse(JB1, method=pseudo):
# Berechnung der Matrix Ui: EE-Geschwindigkeiten -> Koppelpunktgeschwindigkeiten P. i steht f�r den Index des jeweiligen Beines
for i to N_LEGS do
  r||i := P||i:
  if angleConv = X_Y_Z then
     r||i := vec2skew(rotx(xE_t(4)).roty(xE_t(5)).rotz(xE_t(6)).r||i):
  elif angleConv = Z_Y_X then
     r||i := vec2skew(rotz(xE_t(4)).roty(xE_t(5)).rotx(xE_t(6)).r||i):
  end:
  ones := Matrix(3,3,shape=identity):
  U||i := <ones|-r||i>:
  U||i||D := diff~(U||i,t):      #dU berechnen
  #U||i||D := convert_t_s(U||i||D): #dU berechnen
  #U||i := convert_t_s(U||i):
end do:
U_i := Copy(U1):
UD_i := Copy(U1D):
ROW := RowDimension(U1):
COLUMN := ColumnDimension(U1):
# Substituiere die zeitabh�ngigen EE-Koordinaten mit den oben definierten zeitunabh�ngigen Koordinaten
for i to N_LEGS do
  for j to 3 do
    for k to 6 do
      for l from 4 to 6 do
        U||i(j,k) := subs(xE_t(l)=xE_s(l),U||i(j,k)):
        U||i||D(j,k) := subs({xED_t(l)=xED_s(l),xE_t(l)=xE_s(l)},U||i||D(j,k)):
      end do:
    end do:
  end do:
  U_i(1..ROW,1..COLUMN,i) := U||i:
  UD_i(1..ROW,1..COLUMN,i) := U||i||D:
end do:
# Berechnung von JB1D
JB1 := convert_s_t(JB1):
JB1D := diff~(JB1,t):
JB1D := simplify(combine(JB1D)):
JB1D := convert_t_s(JB1D):
JB1 := convert_t_s(JB1):
# Berechne Jacobi-Matrizen f�r jedes Bein
# Dupliziere alle berechneten Matrizen. i steht f�r den Index des jeweiligen Beines
JBinv_i := Copy(JB1inv):
JBD_i := Copy(JB1D):
JB_i := Copy(JB1):
ROW := RowDimension(JB1):
COLUMN := ColumnDimension(JB1):
for i to N_LEGS do
  JB||i||D := Copy(JB1D):
  JB||i||inv := Copy(JB1inv):
  JB||i := Copy(JB1):
end do:
NQJ_parallel;
# Substituiere in jeder Matrix den Winkel Alpha (Verdrehung in der Basis) und die Gelenkkoordinaten und -geschwindigkeiten
for k from 1 by 1 to N_LEGS do  
	for i to ROW do
		for j to COLUMN do
			for l to 3 do
				JB||k||D(i,j):=subs({frame_A_i(l,1)=frame_A_i(l,k)},JB||k||D(i,j)):
				JB||k||inv(j,i):=subs({frame_A_i(l,1)=frame_A_i(l,k)},JB||k||inv(j,i)):
				JB||k(i,j):=subs({frame_A_i(l,1)=frame_A_i(l,k)},JB||k(i,j)):
			end do:
      		for m to NQJ_parallel do #alpha
				#tmp := VARS(m):
        			n := m + (k-1)*NQJ_parallel:
        			JB||k||D(i,j):=subs({qJD_i_s(m,1)=qJD_i_s(m,k),qJ_i_s(m,1)=qJ_i_s(m,k)},JB||k||D(i,j)):
       			JB||k||inv(j,i):=subs({qJ_i_s(m,1)=qJ_i_s(m,k)},JB||k||inv(j,i)):
        			JB||k(i,j):=subs({qJ_i_s(m,1)=qJ_i_s(m,k)},JB||k(i,j)):
     		end do:
    		end do:
  	end do:
  	JB_i(1..ROW,1..COLUMN,k) := JB||k:
  	JBD_i(1..ROW,1..COLUMN,k) := JB||k||D:
  	JBinv_i(1..COLUMN,1..ROW,k) := JB||k||inv:
end do:
# Gesamt Jacobi-Matrix
# Berechnung der inv. Jacobi-Matrix: EE-Geschwindigkeiten -> aktive Gelenkgeschwindigkeiten
Tmp := JB1inv[AKTIV,1..3].U1:
for i from 2 to N_LEGS do
  Tmp := <Tmp;JB||i||inv[AKTIV,1..3].U||i>:
end do:
Jinv := Tmp:
IdentMat := IdentityMatrix(6,6):
counter := 0:
for i to 6 do
  if not(Equal(Matrix(Jinv(1..N_LEGS, i)), ZeroMatrix(N_LEGS,1))) then
    if counter = 0 then
      counter := 1;
      JinvRed := Column(Jinv, i);
      pivotMat := IdentMat(i,..);
    else 
      JinvRed := <JinvRed|Column(Jinv, i)>;
      pivotMat := <pivotMat;IdentMat(i,..)>;
    end if;
  end if:
end do:
Jinv := JinvRed:
# Export
# Maple-Export
save pivotMat, P_i, Jinv, JB_i, JBD_i, JBinv_i, U_i, UD_i, sprintf("../codeexport/%s/tmp/kinematics_%s_platform_maple.m", robot_name, base_method_name):
MatlabExport(Jinv, sprintf("../codeexport/%s/tmp/Jinv_para_matlab.m", robot_name), codegen_opt):
