
# Kinematik für parallelen Roboter
# Einleitung
# Berechnung der Jacobi-Matrix des parallelen Roboters
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# para -> Berechnung für eine parallelen Roboter
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
#interface(warnlevel=0): # Unterdrücke die folgende Warnung.
restart: # Gibt eine Warnung, wenn über Terminal-Maple mit read gestartet wird.
#interface(warnlevel=3):
with(LinearAlgebra):
#with(ArrayTools):
with(codegen):
with(CodeGeneration):
with(StringTools):
# Einstellungen für Code-Export: Optimierungsgrad (2=höchster) und Aktivierung jedes Terms.
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
read "../robot_codegen_definitions/robot_env_par":
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", leg_name):
# Definitionen für parallel Roboter laden
read "../robot_codegen_definitions/robot_env_par":
read sprintf("../codeexport/%s/tmp/para_definitions", robot_name):
# Lade "robotics_repo_path"-File mit Link zum "imes-robotics-matlab"-Repo
read("../robotics_repo_path"):
# Lade die Funktionen aus dem "imes-robotics-matlab"-Repo
read(sprintf("%s/transformation/maple/proc_eul%s2r", robotics_repo_path, angleConvLeg)):
# Kennung des Parametersatzes, für den die Dynamikfunktionen erstellt werden sollen. Muss im Repo und in der mpl-Datei auf 1 gelassen werden, da die folgende Zeile mit einem Skript verarbeitet wird.
codegen_dynpar := 2:
# Link-Index, für den die Jacobi-Matrix aufgestellt wird. Hier wird angenommen, dass der Endeffektor das letzte Segment (=Link) ist. Die Jacobi-Matrix kann hier aber für beliebige Segmente aufgestellt werden. (0=Basis)
LIJAC:=NL-1:
# Ergebnisse der analytischen Jacobi-Matrix (Translatorisch)
read sprintf("../codeexport/%s/tmp/jacobia_transl_%d_maple.m", leg_name, LIJAC):
read "../robot_codegen_definitions/robot_env_par":
b_transl := b_transl:
px, py, pz := 0, 0, 0:
alphaxs_base, betays_base, gammazs_base := 0, 0, 0: 
# Additional Kinematics
# Definition der Koppelpunkte für jedes Bein und der EE-Koordinaten/-Geschwindigkeiten/-Beschleunigungen
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
transDOF := nops(indets(xE_s(1..3,1))):
JB1 := (parse(sprintf("eul%s2r",angleConvLeg))(frame_A_i(1..3,1)).b_transl)(1..transDOF,1..NQJ_parallel):
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
JB1inv := MatrixInverse(JB1):
for i from 1 to 3-transDOF do
  JB1inv := <JB1inv|ZeroMatrix(NQJ_parallel,1)>;
end do:
for i from 1 to 3-transDOF do
  JB1 := <JB1;ZeroMatrix(1,NQJ_parallel)>;
end do:
# Berechnung der Matrix Ui: EE-Geschwindigkeiten -> Koppelpunktgeschwindigkeiten P. i steht für den Index des jeweiligen Beines
for i to N_LEGS do
  r||i := P||i:
  if angleConv = X_Y_Z then
     r||i := vec2skew(rotx(xE_t(4)).roty(xE_t(5)).rotz(xE_t(6)).r||i):
  elif angleConv = Z_Y_X then
     r||i := vec2skew(rotz(xE_t(4)).roty(xE_t(5)).rotx(xE_t(6)).r||i):
  end:
  
  r||i := vec2skew(parse(sprintf("eul%s2r",angleConvLeg))(xE_t(4..6)).P||i);
  ones := Matrix(3,3,shape=identity):
  U||i := <ones|-r||i>:
  U||i||D := diff~(U||i,t):      #dU berechnen
  #U||i||D := convert_t_s(U||i||D): #dU berechnen
  #U||i := convert_t_s(U||i):
end do:

robotType := 1:
counter := 0:
for i from 4 to 6 do
  if xE_t(i) = 0 then
    counter := counter + 1:
  end if:
end do:
if counter = 2 then
  robotType := 2:
  for i from 1 to 3 do
    if not(xE_t(i+3) = 0) then
      rotPlanar := substring(angleConvLeg,i);
    end if:
  end do:
elif counter = 3 then
  robotType := 3:
end if:
robotType:
rotPlanar:

U_i := Copy(U1):
UD_i := Copy(U1D):
ROW := RowDimension(U1):
COLUMN := ColumnDimension(U1):
# Substituiere die zeitabhängigen EE-Koordinaten mit den oben definierten zeitunabhängigen Koordinaten
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
# Berechne Jacobi-Matrizen für jedes Bein
# Dupliziere alle berechneten Matrizen. i steht für den Index des jeweiligen Beines
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
NQJ_parallel:
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
Tmp := simplify(JB1inv[AKTIV,1..3].U1):
for i from 2 to N_LEGS do
  Tmp := <Tmp;JB||i||inv[AKTIV,1..3].U||i>:
end do:
Jinv := Tmp:
IdentMat := IdentityMatrix(6,6):
IdentMatMas := IdentityMatrix(6,6):
counter := 0:
for i to 3 do
  if not(xE_t(i) = 0) then
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
if robotType = 2 then
  if rotPlanar = z then
    JinvRed := <JinvRed|Column(Jinv, 6)>;
    pivotMat := <pivotMat;IdentMat(6,..)>;
  elif rotPlanar = y then
    JinvRed := <JinvRed|Column(Jinv, 5)>;
    pivotMat := <pivotMat;IdentMat(5,..)>;
  elif rotPlanar = x then
    JinvRed := <JinvRed|Column(Jinv, 4)>;
    pivotMat := <pivotMat;IdentMat(4,..)>;
  end if;
elif robotType = 1 then
  if counter = 0 then
    JinvRed := Jinv(..,1..3);
    pivotMat := IdentMat(4..6,..);
  else
    JinvRed := <JinvRed|Jinv(..,1..3)>;
    pivotMat := <pivotMat;IdentMat(4..6,..)>;
  end if;
end if;
counter := 0:
for i from 1 to 6 do
  if not(xE_t(i) = 0) then
    if counter = 0 then
      counter := 1;
      pivotMatMas := IdentMat(i,..);
    else 
      pivotMatMas := <pivotMatMas;IdentMat(i,..)>;
    end if;
  end if:
end do:
Jinv := JinvRed:
# Export
# Maple-Export
save pivotMat, pivotMatMas, P_i, Jinv, JB_i, JBD_i, JBinv_i, U_i, UD_i, sprintf("../codeexport/%s/tmp/kinematics_%s_platform_maple.m", robot_name, base_method_name):
MatlabExport(Jinv, sprintf("../codeexport/%s/tmp/Jinv_para_matlab.m", robot_name), codegen_opt):

