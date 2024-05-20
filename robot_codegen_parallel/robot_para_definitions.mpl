
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
with(ArrayTools): # Für HasNonZero
;
with(StringTools): # Für Zeitausgabe
;
read "../helper/proc_MatlabExport":
read "../helper/proc_convert_t_s":
read "../helper/proc_vector2symmat":
# Lese Umgebungsvariable für Codegenerierung.
read "../robot_codegen_definitions/robot_env_par":
printf("Generiere Parameter für %s\n",robot_name):
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", leg_name):
# Ergebnisse der analytischen Jacobi-Matrix (Translatorisch)
# Link-Index, für den die Jacobi-Matrix aufgestellt wird. Hier wird angenommen, dass der Endeffektor das letzte Segment (=Link) ist. Die Jacobi-Matrix kann hier aber für beliebige Segmente aufgestellt werden. (0=Basis)
LIJAC:=NL-1:
px, py, pz := 0, 0, 0:
alphaxs_base, betays_base, gammazs_base := 0, 0, 0: 
legjacobifile := sprintf("../codeexport/%s/tmp/jacobia_transl_%d_maple.m", leg_name, LIJAC):
if FileTools[Exists](legjacobifile) then
  read legjacobifile:
else
  printf("%s. Beinketten-Jacobi-Matrix konnte nicht geladen werden. Abbruch der Berechnung.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  quit: # Funktioniert in GUI nicht richtig...
  robot_name := "": # ...Daher auch Löschung des Roboternamens.
end if:
b_transl := b_transl: # Variable neu definieren, damit sie sichtbar ist.
# Lese Umgebungsvariable für Codegenerierung (falls nicht oben abgebrochen wurde).
if robot_name <> "" then
  read "../robot_codegen_definitions/robot_env_par":
end if:
# Additional Definitions
# Parameter definieren, wenn nicht vorher schon geschehen
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
# Winkelkonventionen
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
# Dynamik-Parameter
# Einzelne Dynamikparameter als Matlab-Code exportieren. Wenn die Parameter (der Beinkette) durch Benutzereingaben (oder aus PKM-Datenbank) verändert wurden, lässt sich diese Information so weiter benutzen.
# (z.B. in der Definition von Eingabeparametern in den Testskripten). Siehe gleichlautender Abschnitt in robot_tree_floatb_twist_definitions.mw.

# In den Maple-Variablen ist die Spalte der Index der Körper und die Zeilen sind die Indizes für die xyz-Komponenten (einfacherer Aufruf der Variablen).
# In Matlab ist es umgekehrt (führt zu konsistenterem Code in Matlab): Die Zeilen entsprechen dem Körper-Index. Daher hier Transponierung.
# Zusätzlich ist die Reihenfolge der Komponenten der Trägheitstensoren in Matlab und Maple unterschiedlich (daher die Indizierung).
# Matlab: xx, yy, zz, xy, xz, yz (erst Hauptmomente, dann Deviationsmomente)
# Maple: xx, xy, xz, yy, yz, zz (Dreiecksform)
# Masse
mP_generic := parse("MP"):
M_plf := copy(mP_generic):
if assigned(user_M_plf) then
  if not type(user_M_plf, 'scalar') then
    printf("Input user_M_plf is not Scalar. Error.\n"):
  end if:
  # Mass parameter given by user.
    if user_M_plf<>mP_generic and user_M_plf<>0 then # check input
      printf("User input for platform mass should be either \"%s\" or zero. Not \"%s\". Continue anyway. \n", mP_generic, user_M_plf):
    end if:
    M_plf := user_M_plf:
  printf("Plattform-Masseparameter von Benutzer gesetzt. Eingabe für M_plf:\n"):
  print(M_plf);
end if:
# Schwerpunkt
r_P_P_SP := Matrix(3,1):
r_P_P_SP_generic := Matrix(<parse("SXP"), parse("SYP"), parse("SZP")>):

if not assigned(user_CoM_plf) then
  r_P_P_SP := r_P_P_SP_generic:
else
  if RowDimension(user_CoM_plf) <> 3 or ColumnDimension(user_CoM_plf) <> 1 then
    printf("Input user_CoM_plf is not of size 3x1. Error.\n"):
  end if:
    for j from 1 to 3 do
      if user_CoM_plf[j,1]<>r_P_P_SP_generic[j,1] and user_CoM_plf[j,1]<>0 then # check input
        printf("User input for platform CoM %d should be either \"%s\" or zero. Not \"%s\". Continue anyway. \n", j, r_P_P_SP_generic[j,1], convert(user_CoM_plf[j,1],string)):
      end if:
    end do:
    r_P_P_SP[1..3,1]:=user_CoM_plf[1..3,1]:
  printf("Plattform-Schwerpunktsparameter von Benutzer gesetzt. Eingabe für r_P_P_SP:\n"):
  print(r_P_P_SP):
end if:
# Trägheitstensor
I_P_SP := Matrix(6, 1):
I_P_SP_generic := Matrix(6,1,[XXCP,XYCP,XZCP,YYCP,YZCP,ZZCP]):
if not assigned(user_inertia_plf) then
  if M_plf <> 0 then # Zero Mass can be set by the user. Then the inertia stays zero
    I_P_SP := I_P_SP_generic:
  end if:
else
  if RowDimension(user_inertia_plf) <> 6 or ColumnDimension(user_inertia_plf) <> 1 then
    printf("Input user_inertia_plf is not of size 6x1. Error.\n"):
  end if:
    for j from 1 to 6 do
      if user_inertia_plf[j,1]<>I_P_SP_generic[j,1] and user_inertia_plf[j,1]<>0 then # check input
        printf("User input for platform inertia %d should be either \"%s\" or zero. Not \"%s\". Continue anyway. \n", j, convert(I_P_SP_generic[j,1],string), convert(user_inertia_plf[j,1],string)):
      end if:
    end do:
    if M_plf = 0 and HasNonZero(user_inertia_plf) then
    	 printf("Platform mass is zero, but inertia is non-zero. Not plausible, but continue anyway.\n"):
    end if:
    I_P_SP[1..6,1]:=user_inertia_plf[1..6,1]:
  printf("Plattform-Trägheitsparameter von Benutzer gesetzt. Eingabe für I_P_SP:\n"):
  print(I_P_SP):
end if:
# First Moment (mass and center of mass)
mr_P_P_SP := Matrix(3, 1):
mr_P_P_SP_generic := Matrix(3, 1):
xyzstrings := ["X", "Y", "Z"]:
for j from 1 to 3 do # loop over x-, y-, z-coordinates of the CoM
  mr_P_P_SP_generic[j,1]:= parse(sprintf("M%sP", xyzstrings[j])):
end do:
if M_plf <> 0 then: # Zero Mass can be set by the user. Then the first moment stays zero
  for j from 1 to 3 do # loop over x-, y-, z-coordinates of the CoM
    if r_P_P_SP[j,1] = parse(sprintf("S%sP", xyzstrings[j])) then
    	 # default value for CoM is set (not overwritten by user input).
    	 # Set default value for first moment
      mr_P_P_SP[j,1]:= parse(sprintf("M%sP", xyzstrings[j])):
    else
      # CoM is written by the user. Put this assumption also in the first moment to reduce dynamics parameters
    	 mr_P_P_SP[j,1] := M_plf*r_P_P_SP[j,1]:
    end if:
  end do:
end if:
# Inertia (about the origin of body frame, in link frame)
I_P_P_calc := Matrix(6, 1):
# Trägheitstensor (3x3) um den Körperschwerpunkt in Körper-KS
I_P_SP_Tensor := vec2symmat(I_P_SP([1, 2, 4, 3, 5, 6], 1), 3): # Reihenfolge in I_P_SP und vec2symmat unterschiedlich definiert
# Steinerschen Satz anwenden
I_P_P_Tensor := I_P_SP_Tensor + M[i,1]*Transpose(vec2skew(r_P_P_SP[1..3,1])) . vec2skew(r_P_P_SP[1..3,1]):
# Wieder als Matrix abspeichern
I_P_P_calc := <I_P_P_Tensor[1,1]; I_P_P_Tensor[1,2]; I_P_P_Tensor[1,3]; I_P_P_Tensor[2,2]; I_P_P_Tensor[2,3]; I_P_P_Tensor[3,3]>:
# Allgemeine Form des Trägheitstensors (Einträge sind unabhängige Parameter)
I_P_P_generic := Matrix(6, 1):
I_P_P_generic[1,1]:=parse(sprintf("XXP")):
I_P_P_generic[2,1]:=parse(sprintf("XYP")):
I_P_P_generic[3,1]:=parse(sprintf("XZP")):
I_P_P_generic[4,1]:=parse(sprintf("YYP")):
I_P_P_generic[5,1]:=parse(sprintf("YZP")):
I_P_P_generic[6,1]:=parse(sprintf("ZZP")):
# Prüfe, welche Einträge des Trägheitstensors noch Schwerpunkts-Parameter enthalten. Diese müssen wieder auf die Standard-Werte mit unabhängigen Parametern gesetzt werden
I_P_P := Matrix(6, 1):
compstrings := ["XX", "XY", "XZ", "YY", "YZ", "ZZ"]:
if M_plf <> 0 then # Zero Mass can be set by the user. Then the inertia stays zero
  for j from 1 to 6 do # Alle Komponenten des Tensors
    # Nicht Null. Setze erstmal allgemeinen Eintrag:
    I_P_P[j,1] := I_P_P_generic[j,1]:
    # Prüfe, ob Schwerpunktsparameter vorkommt (durch Verschiebungssatz s.o.)
    IhasCoM := false:
    for k from 1 to 3 do
      if has(I_P_P_calc[j,1], r_P_P_SP_generic[k,1]) then
    	   IhasCoM := true: # Der Eintrag enthält die Schwerpunktskoordinaten. Nicht zulässig für weitere Rechnung.
    	   break:
      end if:
    end do:
    if has(I_P_P_calc[j,1], I_P_SP_generic[j,1]) then
    	 IhasCoM := true: # Es steht der Schwerpunktsbezogene Trägheitsterm drin. Weitere Rechnung damit nicht möglich.
    end if:
    if IhasCoM = true then
      next: # Es kommt ein Schwerpunktsparameter vor. Belasse Parameter auf allgemeinem Wert.
    end if:
    I_P_P[j,1] := I_P_P_calc[j,1]:
  end do:
end if:
# Die Dynamikparameter werden nur bis zum Körper vor dem Koppelgelenk später in den Funktionen benötigt. Können noch durch Benutzereingabe zu Null gesetzt werden
M_PKM := <M(2..NQJ_parallel+1,1), M_plf>:
rC_PKM := <r_i_i_Si(1..3,2..NQJ_parallel+1)|r_P_P_SP>:
mrC_PKM := <mr_i_i_Si(1..3,2..NQJ_parallel+1)|mr_P_P_SP>:
IC_PKM := <I_i_Si(1..6,2..NQJ_parallel+1)|I_P_SP>:
IF_PKM := <I_i_i(1..6,2..NQJ_parallel+1)|I_P_P>:
# Export für Testskript, damit zu Null gesetzt Parameter dort auch zu Null gesetzt werden
MatlabExport(M_PKM, sprintf("../codeexport/%s/tmp/parameters_dyn_mges_pkm_matlab.m", robot_name), 2):
MatlabExport(Transpose(rC_PKM), sprintf("../codeexport/%s/tmp/parameters_dyn_rSges_pkm_matlab.m", robot_name), 2):
MatlabExport(Transpose(IC_PKM([1,4,6,2,3,5],..)), sprintf("../codeexport/%s/tmp/parameters_dyn_Icges_pkm_matlab.m", robot_name), 2):
MatlabExport(Transpose(mr_i_i_Si), sprintf("../codeexport/%s/tmp/parameters_dyn_mrSges_pkm_matlab.m", robot_name), 2):
MatlabExport(Transpose(I_i_i([1,4,6,2,3,5],..)), sprintf("../codeexport/%s/tmp/parameters_dyn_Ifges_pkm_matlab.m", robot_name), 2):
# Export
# Maple-Export
save g_world, I_P_P, mr_P_P_SP, NX, NQJ_parallel, angleConvLeg, angleConv, r_P_P_SP, frame_A_i, qJ_i_s, qJD_i_s, qJDD_i_s, xE_t, xED_t, xEDD_t, xE_s, xED_s, xEDD_s, I_P_SP, M_plf, sprintf("../codeexport/%s/tmp/para_definitions", robot_name):
varScript := <NX;NQJ_parallel;legAngles>:
MatlabExport(varScript, sprintf("../codeexport/%s/tmp/var_parallel.m", robot_name), 1);
# Erzeuge weitere Variablen, die für die Erzeugung von Template-Funktionen aus der Matlab-Robotik-Toolbox benötigt werden
I_EE := Matrix(1,6): # Zähler für aktive Endeffektor-Koordinaten
for k from 1 to 6 do
  if xE_t(k,1) <> 0 then
    I_EE(1,k) := 1:
  end if:
end do:
I1J_LEG := Matrix(1,N_LEGS): # Anfangs-Index der Gelenkkoordinaten der Beinketten in allen Koordinaten
I2J_LEG := copy(I1J_LEG): # End-Indizes
Leg_NQJ := copy(I1J_LEG): # Anzahl der Gelenk-Koordinaten der Beinketten (inklusive Koppelgelenke)
Leg_NL := copy(I1J_LEG): # Anzahl der Starrkörper der Beinketten (inkl. Basis)
for k from 1 to N_LEGS do
  if k = 1 then
    I1J_LEG(1) := 1:
  else
    I1J_LEG(k) := I2J_LEG(k-1) + 1:
  end if:
  I2J_LEG(k) := I1J_LEG(k) + NJ - 1:
  Leg_NQJ(k) := I2J_LEG(k)-I1J_LEG(k)+1:
  Leg_NL(k) := Leg_NQJ(k) + 1;
end do:
NJ_PKM := I2J_LEG(N_LEGS):
NL_PKM := 1+NJ_PKM+1: # zusätzlich ein Körper für Basis und für Plattform
save I_EE, I1J_LEG, I2J_LEG, Leg_NQJ, Leg_NL, NJ_PKM, NL_PKM, sprintf("../codeexport/%s/tmp/para_definitions_for_templatefcns", robot_name):

