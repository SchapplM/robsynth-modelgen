
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
read "../transformation/proc_transl": 
read "../transformation/proc_trafo_mdh": 
read "../robot_codegen_definitions/robot_env":
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name):
# Kennung des Parametersatzes, für den die Dynamikfunktionen erstellt werden sollen. Muss im Repo und in der mpl-Datei auf 1 gelassen werden, da die folgende Zeile mit einem Skript verarbeitet wird.
codegen_dynpar := 1:
# Link-Index, für den die Jacobi-Matrix aufgestellt wird. Hier wird angenommen, dass der Endeffektor das letzte Segment (=Link) ist. Die Jacobi-Matrix kann hier aber für beliebige Segmente aufgestellt werden. (0=Basis)
LIJAC:=NL-1:
# Ergebnisse der analytischen Jacobi-Matrix (Translatorisch)
read sprintf("../codeexport/%s/tmp/jacobia_transl_%d_maple.m", robot_name, LIJAC):
JB1 := b_transl:
printf("Generiere Dynamik für PKM %s mit Parametersatz %d und %s\n", robot_name, codegen_dynpar, base_method_name):
# Additional Kinematics
# Erstelle EE-Koordinaten: xE_t, xED_t, xEDD_t -> Variablen mit Zeitabhängigkeit
#                                          xE_s, xED_s, xEDD_s -> Variablen ohne Zeitabhängigkeit
xE_t:=<xE(t);yE(t);zE(t);PsiE(t);ThetaE(t);PhiE(t)>:
xED_t:=diff~(xE_t,t):
xEDD_t:=diff~(xED_t,t):
xED_s := copy(xE_s):
xEDD_s := copy(xE_s):
FHG_trans := 0:FHG_rot := 0:
Counter := Matrix(6,1):
for i to 6 do
   if not(xE_s(i) = 0) then
   	 Tmp := xE_s(i):
      xED_s(i) := D||Tmp:
      xEDD_s(i) := DD||Tmp:
   else
      xED_s(i) := 0:
      xEDD_s(i) := 0:
      xE_t(i) := 0:
      xED_t(i) := 0:
      xEDD_t(i) := 0:
   end if:
end do:
# Definition der Koppelpunkte für jedes Bein und der EE-Koordinaten/-Geschwindigkeiten/-Beschleunigungen
for i to N_LEGS do
  P||i := <xP[i];yP[i];zP[i]>:
end do:
px, py, pz := 0, 0, 0:
# Jacobi Matrices (JB1/U1) + Derivates
# Berechnung der Jacobi-Matrix JB1inv: Gelenkgeschwindigkeiten -> Koppelpunktgeschwindigkeiten P
for i to 3 do
	if JB1(1,i) = 0 and JB1(2,i) = 0 and JB1(3,i) = 0
	and JB1(i,1) = 0 and JB1(i,2) = 0 and JB1(i,3) = 0 then
		JB1(i,i) := 1:	end if:
end do:
# Berechnung der Jacobi-Matrix JBE: Koppelpunktgeschwindigkeiten P -> Gelenkgeschwindigkeiten
JB1inv := MatrixInverse(JB1):
# Berechnung der Matrix Ui: EE-Geschwindigkeiten -> Koppelpunktgeschwindigkeiten P. i steht für den Index des jeweiligen Beines
for i to N_LEGS do
  r||i := P||i:
  r||i := rotx(0).roty(0).rotz(xE_t(6)).vec2skew(r||i):
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
# Substituiere in jeder Matrix den Winkel Alpha (Verdrehung in der Basis) und die Gelenkkoordinaten und -geschwindigkeiten
for k from 1 by 1 to N_LEGS do  for i to NQJ do
    for j to NQJ do
    	 JB||k||D(i,j):=subs({alpha=alpha[k]},JB||k||D(i,j)):
    	 JB||k||inv(i,j):=subs({alpha=alpha[k]},JB||k||inv(i,j)):
    	 JB||k(i,j):=subs({alpha=alpha[k]},JB||k(i,j)):
      for m to NQJ do #alpha
        #tmp := VARS(m):
        n := m + (k-1)*NQJ:
        JB||k||D(i,j):=subs({qJD||m||s=qJ||D||n||s,qJ||m||s=qJ||n||s},JB||k||D(i,j)):
        JB||k||inv(i,j):=subs({qJ||m||s=qJ||n||s},JB||k||inv(i,j)):
        JB||k(i,j):=subs({qJ||m||s=qJ||n||s},JB||k(i,j)):
      end do:
    end do:
  end do:
  JB_i(1..ROW,1..COLUMN,k) := JB||k:
  JBD_i(1..ROW,1..COLUMN,k) := JB||k||D:
  JBinv_i(1..ROW,1..COLUMN,k) := JB||k||inv:
end do:
# Gesamt Jacobi-Matrix
# Berechnung der inv. Jacobi-Matrix: EE-Geschwindigkeiten -> aktive Gelenkgeschwindigkeiten
Tmp := JB1inv[AKTIV,1..3].U1:
for i from 2 to N_LEGS do
  Tmp := <Tmp;JB||i||inv[AKTIV,1..3].U||i>:
end do:
Jinv := Tmp:
# Export
# Maple-Export
save Jinv, JB_i, JBD_i, JBinv_i, U_i, UD_i, xE_t, xED_t, xEDD_t, xE_s, xED_s, xEDD_s, sprintf("../codeexport/%s/tmp/kinematics_%s_platform_maple.m", robot_name, base_method_name):
MatlabExport(Jinv, sprintf("../codeexport/%s/tmp/Jinv_para_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
# Matlab Export: Floating base
# Berechnung der Basis-Belastung ist für manche Basis-Darstellungen falsch (siehe oben unter Gravitationslast).
if codeexport_invdyn and not(base_method_name="twist") then
  MatlabExport(tau, sprintf("../codeexport/%s/tmp/invdyn_para_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
# Matlab Export: Fixed base
if codeexport_invdyn then
  taus_fixb:=tauGes:
  for i from 1 to NQB do
    taus_fixb := subs({X_base_s[i,1]=0},taus_fixb):
  end do:
  for i from 1 to 6 do
    taus_fixb := subs({V_base_s[i,1]=0},taus_fixb):
    taus_fixb := subs({VD_base_s[i,1]=0},taus_fixb):
  end do:
  save taus_fixb, sprintf("../codeexport/%s/tmp/invdyn_para_fixb_par%d_maple.m", robot_name, codegen_dynpar):
end if:

