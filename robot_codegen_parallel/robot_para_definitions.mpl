
# Definitions for Parallel Robot Dynamics Code Generation
# Einleitung
# Erstelle Definitionen für die Maple-Skripte zur Berechnung von Kinematik und Dynamik des parallelen Roboters
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# para -> Berechnung für eine parallelen Roboter
# definitions -> Definitionen
# Autor
# Tim Job (Studienarbeit bei Moritz Schappler), 2018-12
# Moritz Schappler, moritz.schappler@imes.uni-hannover.de
# (C) Institut für Mechatronische Systeme, Universität Hannover
# Init
interface(warnlevel=0): # Unterdrücke die folgende Warnung.
restart: # Gibt eine Warnung, wenn über Terminal-Maple mit read gestartet wird.
interface(warnlevel=3):
with(LinearAlgebra):
read "../helper/proc_MatlabExport":
read "../helper/proc_convert_t_s":
# Lese Umgebungsvariable für Codegenerierung.
read "../robot_codegen_definitions/robot_env_par":
printf("Generiere Parameter für %s\n",robot_name):
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", leg_name):
# Link-Index, für den die Jacobi-Matrix aufgestellt wird. Hier wird angenommen, dass der Endeffektor das letzte Segment (=Link) ist. Die Jacobi-Matrix kann hier aber für beliebige Segmente aufgestellt werden. (0=Basis)
LIJAC:=NL-1:
# Ergebnisse der analytischen Jacobi-Matrix (Translatorisch)
px, py, pz := 0, 0, 0:
alphaxs_base, betays_base, gammazs_base := 0, 0, 0: 
read sprintf("../codeexport/%s/tmp/jacobia_transl_%d_maple.m", leg_name, LIJAC):
b_transl := b_transl:
# Lese Umgebungsvariable für Codegenerierung.
read "../robot_codegen_definitions/robot_env_par":
# Additional Definitions
# Parameter definieren, wenn nicht vorher schon geschehen
if not assigned(J_SP) then
   J_SP := Matrix(3,3,[XX,XY,XZ,XY,YY,YZ,XZ,YZ,ZZ]):
end if:
if not assigned(J_P_P) then
   J_P_P := Matrix(3,3,[XXFP,XYFP,XZFP,XYFP,YYFP,YZFP,XZFP,YZFP,ZZFP]):
end if:
if not assigned(xE_s) then
   xE_s:=<x_all[1];x_all[2];x_all[3];x_all[4];x_all[5];x_all[6]>:
end if:

# Gravity vector in world frame
if not assigned(g_world) then
  g_world := Matrix(3, 1):
  g_world(1 .. 3, 1) := <g1, g2, g3>:
end if:

# Parallel Robotics Definitions
# Erstelle für Gelenkkoordinaten, -geschwindigkeiten und -beschleunigungen für jedes Bein
J := simplify(b_transl):
vars := indets(J):
counter := 0:
thetaList := convert(vars,list):
dList := convert(d,list):
for i to NQJ do
  var := parse(sprintf("qJ%ds", i)):
  if has(thetaList, var) then
    counter := counter + 1:
  end if:
  if has(dList, var) then
    counter := counter + 1:
  end if:
end do:
NQJ_parallel := counter:
counter := 0:
for i to ColumnDimension(J) do
  if not(Equal(J(..,i),Vector(3,[0,0,0]))) then
    counter := counter + 1:
  end if:
end do:
NQJ_parallel := counter:

qJ_i_s := Matrix(NQJ_parallel,N_LEGS):
qJD_i_s := Matrix(NQJ_parallel,N_LEGS):
qJDD_i_s := Matrix(NQJ_parallel,N_LEGS):
for i from 1 to NQJ_parallel do
  for j from 1 to N_LEGS do
     k := j-1:
     qJ_i_s(i,j):=parse(sprintf("qJ%ds", i+k*NQJ_parallel)):
     qJD_i_s(i,j):=parse(sprintf("qJD%ds", i+k*NQJ_parallel)):
     qJDD_i_s(i,j):=parse(sprintf("qJDD%ds", i+k*NQJ_parallel)):
  end do:
end do:
# Erstelle Vektor der Basisdrehungen jedes Beines
frame_A_i := Matrix(3,N_LEGS):
for i to N_LEGS do
   for j to 3 do
      if has(indets(leg_frame(4..6)),leg_frame(j+3)) then
         frame_A_i(j,i) := leg_frame(j+3)[i]:
      else 
         frame_A_i(j,i) := leg_frame(j+3):
      end if:
   end do:
end do:
# Plattformschwerpunkt
r_P_sP := Vector(3,symbol=r_sP):
s_P_sP := Vector(3,symbol=s_sP):
angleConvLeg := leg_frame(7):
angleConv := xE_s(7):
# Erstelle EE-Koordinaten: xE_t, xED_t, xEDD_t -> Variablen mit Zeitabhängigkeit
#                                          xE_s, xED_s, xEDD_s -> Variablen ohne Zeitabhängigkeit
xE_t:=<xE(t);yE(t);zE(t);PsiE(t);ThetaE(t);PhiE(t)>:
xED_t:=diff~(xE_t,t):
xEDD_t:=diff~(xED_t,t):
# Zähle Freiheitsgrade des Roboters und setze nicht benötigte zu null.
xE_s := Matrix(xE_s(1..6,1)):
xED_s := copy(xE_s):
xEDD_s := copy(xE_s):
FHG_trans := 0:FHG_rot := 0:
counter := 6:
for i to 6 do
   if not(xE_s(i) = 0) then
   	 Tmp := xE_s(i):
      xED_s(i) := D||Tmp:
      xEDD_s(i) := DD||Tmp:
   else
   	 counter := counter - 1:
      xED_s(i) := 0:
      xEDD_s(i) := 0:
      xE_t(i) := 0:
      xED_t(i) := 0:
      xEDD_t(i) := 0:
   end if:
end do:
NX := counter:
counter := 0:
for i from 4 to 6 do
	if not(leg_frame(i) = 0) then
		counter := counter + 1:
	end if:
end do:
legAngles := counter:
# Export
# Maple-Export
save g_world, J_P_P, s_P_sP, NX, NQJ_parallel, angleConvLeg, angleConv, r_P_sP, frame_A_i, qJ_i_s, qJD_i_s, qJDD_i_s, xE_t, xED_t, xEDD_t, xE_s, xED_s, xEDD_s, J_SP, sprintf("../codeexport/%s/tmp/para_definitions", robot_name):
varScript := <NX;NQJ_parallel;legAngles>:
MatlabExport(varScript, sprintf("../codeexport/%s/tmp/var_parallel.m", robot_name), 1);

