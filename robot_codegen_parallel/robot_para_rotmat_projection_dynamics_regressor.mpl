
# Berechnung und Projektion der Dynamikgleichungen in Regressorform
# Einleitung
# Berechnung und Projektion der Dynamikgleichungen in Regressorform
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# para -> Berechnung für einen parallelen Roboter
# rotmat -> Kinematik wird mit Rotationsmatrizen berechnet
# projection -> Die Dynamikgleichungen werden auf EE-Koordinaten projiziert
# dynamics -> Berechnung der Dynamik
# regressor -> Regressorform (parameterlinear)
# 
# TODO
# Die Regressorform ist noch nicht in Minimaldarstellung. Außerdem treten Dynamikparameter auf, die bei reduzierten EE-FG (z.B. 2T1R-Bewegung) keinen Einfluss haben.
# 
# Autor
# Tim Job (Studienarbeit bei Moritz Schappler), 2018-12
# Moritz Schappler, moritz.schappler@imes.uni-hannover.de
# (C) Institut für Mechatronische Systeme, Universität Hannover
# Sources
# [GautierKhalil1990] Direct Calculation of Minimum Set of Inertial Parameters of Serial Robots
# [KhalilDombre2002] Modeling, Identification and Control of Robots
# [Ortmaier2014] Vorlesungsskript Robotik I
# [Job2018_S759] Job, T. (Studienarbeit; Betreuer Moritz Schappler): Implementierung einer strukturunabhängigen Dynamikmodellierung für parallelkinematische Maschinen (2018)
# Initialization
interface(warnlevel=0): # Unterdrücke die folgende Warnung.
restart: # Gibt eine Warnung, wenn über Terminal-Maple mit read gestartet wird.
interface(warnlevel=3):
interface(rtablesize=100):
with(LinearAlgebra):
with(codegen):
with(CodeGeneration):
with(StringTools): # Für Zeitausgabe
;
# Einstellungen für Code-Export: Optimierungsgrad (2=höchster) und Aktivierung jedes Terms.
#codegen_act := true: # noch nicht implementiert
codegen_opt := 2:
codeexport_invdyn := true:
codeexport_actcoord := false: # Generierung der Dynamik in Antriebskoordinaten nicht standardmäßig (hoher Rechenaufwand)
;
read "../helper/proc_MatlabExport": 
# Roboter-Definitionen laden
read "../robot_codegen_definitions/robot_env_par":
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", leg_name):
read "../robot_codegen_definitions/robot_env_par":
# Ergebnisse der Plattform-Dynamik in Regressorform laden

dynamicsfile := sprintf("../codeexport/%s/tmp/floatb_%s_platform_dynamic_reg_maple.m", robot_name, base_method_name):
if FileTools[Exists](dynamicsfile) then
  read dynamicsfile:
else
  printf("%s. Plattform-Dynamik (Regressor) konnte nicht geladen werden. Abbruch der Berechnung.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  quit: # Funktioniert in GUI nicht richtig...
  robot_name := "": # ...Daher auch Löschung des Roboternamens.
end if:
# Neu-Definieren, damit Variablen im Workspace auftauchen
paramVecP := paramVecP:
paramVecP_M := paramVecP_M:
A_E := A_E:
H := H:
dH := dH:
M_regmin := M_regmin:
c_regmin := c_regmin:
g_regmin := g_regmin:
# Ergebnisse der Dynamik der Gelenkkette in Regressorform laden

dynamicsfile := sprintf("../codeexport/%s/tmp/invdyn_%s_%s_maple.m", leg_name, "fixb", "regressor_minpar"):
if FileTools[Exists](dynamicsfile) then
  read dynamicsfile:
else
  printf("%s. PKM-Dynamik konnte nicht geladen werden. Abbruch der Berechnung.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  quit: # Funktioniert in GUI nicht richtig...
  robot_name := "": # ...Daher auch Löschung des Roboternamens.
end if:
tau_regressor_s := tau_regressor_s:
tauMM_regressor_s := tauMM_regressor_s:
MMjj_regressor_s := MMjj_regressor_s:
tauC_regressor_s := tauC_regressor_s:
taug_regressor_s := taug_regressor_s:
read "../robot_codegen_definitions/robot_env_par":

# Ergebnisse des Minimalparametervektors der Gelenkkette laden
read sprintf("../codeexport/%s/tmp/minimal_parameter_vector_fixb_maple", leg_name):
Paramvec2 := Paramvec2:
read "../robot_codegen_definitions/robot_env_par": # Nochmal laden, um Standard-Einstellungen überschreiben zu können.
;
# Ergebnisse der zusätzlichen Definitionen für parallele Roboter laden
read sprintf("../codeexport/%s/tmp/para_definitions", robot_name):
printf("Generiere Parameterlineare Form der Dynamik für PKM %s mit Minimal-Parametersatz\n", robot_name):
# Lade "robotics_repo_path"-File mit Link zum "imes-robotics-matlab"-Repo
read("../robotics_repo_path"):
# Lade die Funktionen aus dem "imes-robotics-matlab"-Repo
read(sprintf("%s/transformation/maple/proc_eul%s2r", robotics_repo_path, angleConvLeg)):
read(sprintf("%s/transformation/maple/proc_eul%sjac", robotics_repo_path, "zyx")):


# Berechne Dynamik-Matrizen für alle Beine
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
alphaDx_base := 0:
betaDy_base := 0:
gammaDz_base := 0:
vDxs_base := 0:
vDys_base := 0:
vDzs_base := 0:
alphaDDx_base := 0:
betaDDy_base := 0:
gammaDDz_base := 0:


# Ergebnisse der Kinematik für parallelen Roboter laden


kinematicsfile := sprintf("../codeexport/%s/tmp/kinematics_%s_platform_maple.m", robot_name, base_method_name):
if FileTools[Exists](kinematicsfile) then
  read kinematicsfile:
else
  printf("%s. PKM-Kinematik konnte nicht geladen werden. Abbruch der Berechnung.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  quit: # Funktioniert in GUI nicht richtig...
  robot_name := "": # ...Daher auch Löschung des Roboternamens.
end if:
# Variablen neu laden
pivotMat := pivotMat:
pivotMatMas := pivotMatMas:
P_i := P_i:
Jinv := Jinv:
JB_i := JB_i:
JBD_i := JBD_i:
JBinv_i := JBinv_i:
JBDinv_i := JBDinv_i:
U_i := U_i:
UD_i := UD_i:

# Dynamik-Parameter für virtuelle Segmente nach den Plattform-Koppelgelenken entfernen
NQ := NQ - (NQJ-NQJ_parallel):
Paramvec3:=Paramvec2: # Dynamik-Minimalparametervektor der Beinkette
;
for i from NQJ_parallel+1 to NQJ do
  Paramvec3 := subs({XXC||i = 0}, Paramvec3):
  Paramvec3 := subs({XYC||i = 0}, Paramvec3):
  Paramvec3 := subs({XZC||i = 0}, Paramvec3):
  Paramvec3 := subs({YYC||i = 0}, Paramvec3):
  Paramvec3 := subs({YZC||i = 0}, Paramvec3):
  Paramvec3 := subs({ZZC||i = 0}, Paramvec3):
  Paramvec3 := subs({XX||i  = 0}, Paramvec3):
  Paramvec3 := subs({XY||i  = 0}, Paramvec3):
  Paramvec3 := subs({XZ||i  = 0}, Paramvec3):
  Paramvec3 := subs({YY||i  = 0}, Paramvec3):
  Paramvec3 := subs({YZ||i  = 0}, Paramvec3):
  Paramvec3 := subs({ZZ||i  = 0}, Paramvec3):
  Paramvec3 := subs({SX||i  = 0}, Paramvec3):
  Paramvec3 := subs({SY||i  = 0}, Paramvec3):
  Paramvec3 := subs({SZ||i  = 0}, Paramvec3):
  Paramvec3 := subs({MX||i  = 0}, Paramvec3):
  Paramvec3 := subs({MY||i  = 0}, Paramvec3):
  Paramvec3 := subs({MZ||i  = 0}, Paramvec3):
  Paramvec3 := subs({M||i   = 0}, Paramvec3):
end do:
Paramvec2: # Bein-Dynamik-Parametervektor vor Vereinfachungen
;
Paramvec3: # Bein-Dynamik-Parametervektor nach Vereinfachungen
;
paramVecP: # Plattform-Dynamik-Parametervektor ohne Zusammenfassungen
;
paramVecP_M: # Plattform-Dynamik-Parametervektor mit Zusammenfassung mit Bein-Dynamikparametern
;
# Alle Einträge des Minimalparametervektors, die jetzt Null sind, entfernen
ParamvecNew := Matrix(RowDimension(Paramvec3), 1): # Initialisierung mit maximaler Größe
k := 0:
for j from 1 to RowDimension(Paramvec3) do
  if not Paramvec3(j,1) = 0 then
    k := k + 1:
    ParamvecNew(k,1) := Paramvec3(j,1):
  end if:
end do:
# Neuen Parametervektor für die Beinsegmenten (ohne Plattform)
ParamvecNew := ParamvecNew(1..k,1):
# Dupliziere alle berechneten Matrizen. i steht für den Index des jeweiligen Beines
tauReg := tau_regressor_s(7..NQ,..):
#MatrixEnd := index_symmat2vec(NQJ_parallel,NQJ_parallel,NQJ_parallel):
#MMjjReg := MMjj_regressor_s(1..MatrixEnd,..):
tauMMReg := tauMM_regressor_s(7..NQ,..):
for i to NQ do 
  tauMMReg := subs({qD_s(i, 1) = 0}, tauMMReg):
end do:
tauCReg := tauC_regressor_s(7..NQ,..):
taugReg := taug_regressor_s(7..NQ,..):

# Regressor aus Berechnung für serielle Beinketten. Dort erste sechs Einträge für Basis
g := <g1;g2;g3>:
tmp := <tmp1;tmp2;tmp3>:
Rmat := Transpose(parse(sprintf("eul%s2r",angleConvLeg))(frame_A_i(1..3,1))):
gtmp := Rmat.g:
for i to NQJ_parallel do
  for k to ColumnDimension(tauReg) do
    for j to 3 do 
      tauReg(i,k) := subs({g(j)=tmp(j)},tauReg(i,k)):
      taugReg(i,k) := subs({g(j)=tmp(j)},taugReg(i,k)):
    end do:
    for j to 3 do 
      tauReg(i,k) := subs({tmp(j)=gtmp(j)},tauReg(i,k)):
      taugReg(i,k) := subs({tmp(j)=gtmp(j)},taugReg(i,k)):
    end do:
  end do:
end do:

for i to N_LEGS do
  tau_regressor_s||i := Copy(tauReg):
  tauMM_regressor_s||i := Copy(tauMMReg):
  #MMjj_regressor_s||i := Copy(MMjjReg):
  tauC_regressor_s||i := Copy(tauCReg):
  taug_regressor_s||i := Copy(taugReg):
end do:
COLUMNreg := ColumnDimension(tau_regressor_s):
# 
# Substituiere in jeder Matrix den Winkel Alpha (Verdrehung in der Basis) und die Gelenkkoordinaten und -geschwindigkeiten
for k from 1 by 1 to N_LEGS do
  for i to NQJ_parallel do
    for j to COLUMNreg do
      for p from NQJ_parallel+1 to NQJ do
        tau_regressor_s||k(i,j) := subs({qJDD||p||s=0,qJD||p||s=0,qJ||p||s=0},tau_regressor_s||k(i,j)):
        tauMM_regressor_s||k(i,j) := subs({qJDD||p||s=0,qJD||p||s=0,qJ||p||s=0},tauMM_regressor_s||k(i,j)):
        tauC_regressor_s||k(i,j) := subs({qJDD||p||s=0,qJD||p||s=0,qJ||p||s=0},tauC_regressor_s||k(i,j)):
        taug_regressor_s||k(i,j) := subs({qJDD||p||s=0,qJD||p||s=0,qJ||p||s=0},taug_regressor_s||k(i,j)):
      end do:
      for l to 3 do
        tau_regressor_s||k(i,j) := subs({frame_A_i(l,1)=frame_A_i(l,k)},tau_regressor_s||k(i,j)):
        tauMM_regressor_s||k(i,j) := subs({frame_A_i(l,1)=frame_A_i(l,k)},tauMM_regressor_s||k(i,j)):
        tauC_regressor_s||k(i,j) := subs({frame_A_i(l,1)=frame_A_i(l,k)},tauC_regressor_s||k(i,j)):
        taug_regressor_s||k(i,j) := subs({frame_A_i(l,1)=frame_A_i(l,k)},taug_regressor_s||k(i,j)):
      end do:
      for m to NQJ_parallel do
        n := (m + (k-1)*NQJ_parallel):
        tau_regressor_s||k(i,j) := subs({qJDD||m||s=qJDD||n||s,qJD||m||s=qJD||n||s,qJ||m||s=qJ||n||s},tau_regressor_s||k(i,j)):
        tauMM_regressor_s||k(i,j) := subs({qJDD||m||s=qJDD||n||s,qJD||m||s=qJD||n||s,qJ||m||s=qJ||n||s},tauMM_regressor_s||k(i,j)):
        tauC_regressor_s||k(i,j) := subs({qJDD||m||s=qJDD||n||s,qJD||m||s=qJD||n||s,qJ||m||s=qJ||n||s},tauC_regressor_s||k(i,j)):
        taug_regressor_s||k(i,j) := subs({qJDD||m||s=qJDD||n||s,qJD||m||s=qJD||n||s,qJ||m||s=qJ||n||s},taug_regressor_s||k(i,j)):
      end do:
    end do:
  end do:
end do:


printf("%s. Dynamik-Matrizen bestimmt. Beginne Projektion/Addition.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
# Berechnung, Projektion und Addition der Dynamikgleichungen
# Berechnung der Kräfte/Momente an den Gelenken der jeweiligen Beine und Projektion auf EE-Plattform
A_E := A_E:
U_i := U_i:
JBinv_i := JBinv_i:
for i to N_LEGS do
   A_||i:= simplify(Transpose(U_i(..,..,i)).Transpose(JBinv_i(..,..,i))).simplify(tau_regressor_s||i):
   Mreg_||i:= simplify(Transpose(U_i(..,..,i)).Transpose(JBinv_i(..,..,i))).simplify(tauMM_regressor_s||i):
   Creg_||i:= simplify(Transpose(U_i(..,..,i)).Transpose(JBinv_i(..,..,i))).simplify(tauC_regressor_s||i):
   greg_||i:= simplify(Transpose(U_i(..,..,i)).Transpose(JBinv_i(..,..,i))).simplify(taug_regressor_s||i):
end do:
# Aufsummieren aller Kräfte, projiziert auf EE-Plattform
Tmp := 0:
TmpM := 0:
TmpC := 0:
Tmpg := 0:
for i to N_LEGS do
  Tmp := Tmp + A_||i:
  TmpM := TmpM + Mreg_||i:
  TmpC := TmpC + Creg_||i:
  Tmpg := Tmpg + greg_||i:
end do:
A_j := Tmp:
Mreg_j := TmpM:
Creg_j := TmpC:
greg_j := Tmpg:
# Addiere Inverse Dynamik der Plattform
ROWS := RowDimension(ParamvecNew):
# 
# Neuer Parametervektor für Beine und Plattform
paramMin := <ParamvecNew;paramVecP>: # TODO: Hier stand vorher paramVecP_M. Damit war die Regressorform aber nicht für alle Systeme konsistent. Jetzt ist die parameterlineare Form aber nicht mehr so stark zusammengefasst, wie sie es sein könnte.
;
# Regressormatrix zusammenstellen
A := pivotMat.<A_j(1..6,1..ROWS)|A_E>:
Mreg := pivotMat.<Mreg_j(1..6,1..ROWS)|M_regmin>:
Creg := pivotMat.<Creg_j(1..6,1..ROWS)|c_regmin>:
greg := pivotMat.<greg_j(1..6,1..ROWS)|g_regmin>:
NotNullEntries := 0:
for i to RowDimension(paramMin) do
	if not(paramMin(i,1) = 0 or Equal(Column(A,i), ZeroVector[column](RowDimension(A)))) then
		NotNullEntries := NotNullEntries + 1:
	end if:
end do:
NotNullEntries:
paramMinRed := Matrix(NotNullEntries,1):
ARed := Matrix(RowDimension(A),NotNullEntries):
MregRed := Matrix(RowDimension(A),NotNullEntries):
CregRed := Matrix(RowDimension(A),NotNullEntries):
gregRed := Matrix(RowDimension(A),NotNullEntries):
j := 1:
for i to RowDimension(paramMin) do
	if not(paramMin(i,1) = 0 or Equal(Column(A,i), ZeroVector[column](RowDimension(A)))) then
		paramMinRed(j,1) := paramMin(i,1):
		for k to RowDimension(A) do
			ARed(k,j) := A(k,i):
			MregRed(k,j) := Mreg(k,i):
			CregRed(k,j) := Creg(k,i):
			gregRed(k,j) := greg(k,i):
		end do:
		j := j + 1:
	end if:
end do:
printf("%s. Beginne Ersetzung der Geschwindigkeiten und Bestimmung der Plattform-Terme.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
# Replace Joint Velocities
# Substituiere die Gelenkgeschwindigkeiten über H-, Ui- und JBi-Matrix mit EE-Geschwindikeiten
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
MtoC := copy(MregRed):
for i to NX do
  for k to RowParamMin do
    for j to N_LEGS do
      for l to NQJ_parallel do
        ARed(i,k) := subs({qJDD_i_s(l,j)=C||j(l)},ARed(i,k)):
        MregRed(i,k) := subs({qJDD_i_s(l,j)=C||j(l)},MregRed(i,k)):
        MtoC(i,k) := subs({qJDD_i_s(l,j)=C||j(l)},MtoC(i,k)):
      end do:
    end do:
    for j to N_LEGS do
      for l to NQJ_parallel do
        ARed(i,k) := subs({qJD_i_s(l,j)=z||j(l)},ARed(i,k)):
        MregRed(i,k) := subs({qJD_i_s(l,j)=z||j(l)},MregRed(i,k)):
        MtoC(i,k) := subs({qJD_i_s(l,j)=z||j(l)},MtoC(i,k)):
        CregRed(i,k) := subs({qJD_i_s(l,j)=z||j(l)},CregRed(i,k)):
      end do:
    end do:
    for j to N_LEGS do
      for l to 6 do
        MregRed(i,k) := subs({xED_s(l,1)=0},MregRed(i,k)):
        MtoC(i,k) := subs({xEDD_s(l,1)=0},MtoC(i,k)):
      end do:
    end do:
  end do:
end do:
CregRed := CregRed+MtoC:
tauGes := ARed:#.paramMinRed:
# Dynamik-Regressor in Plattform-Koordinaten (oben berechnet)
tau_x := tauGes:
# Dynamik-Regressor in Antriebs-Koordinaten umrechnen. Nur machen, wenn die Jacobi-Matrix einfach genug ist.
if RowDimension(Jinv) < 5 and codeexport_actcoord then
  read sprintf("../codeexport/%s/tmp/jacobian_maple.m", robot_name): # Annahme, dass die Jacobi-Matrix (aus Invertierung) vorher berechnet wurde. Setzt aktuell das normale Dynamik-Skript voraus.
  J:=J: # Neudefinition, damit Variable im Workspace ist.
  tau_qa:=Transpose(J).tauGes:
  Mreg_qa:=Transpose(J).MregRed:
  Creg_qa:=Transpose(J).CregRed:
  greg_qa:=Transpose(J).gregRed:
end if:

# Erzeuge Minimalkoordinatenvektor zur Erstellung der Massenmatrix durch Ableitungen im folgenden Schritt
xEDD_s_dummy := Matrix(NX,1):
zaehler := 1:
for i to 6 do
	if not(xEDD_s(i) = 0) then
		xEDD_s_dummy(zaehler) := xEDD_s(i):
		zaehler := zaehler + 1:
	end if:
end do:
# Erzeugung der Massenmatrizen
MMregRed := Matrix(NX*NX, RowDimension(paramMinRed)):
MMreg_qa := Matrix(NX*NX, RowDimension(paramMinRed)):
i_rr := 0:
for i to NX do # Zeilenindex der Massenmatrix
  for j to NX do  # Spaltenindex der Massenmatrix
    #if j > i then
    #  next: # rechte obere Seite der symmetrischen Matrix. Keine neue Information. Nicht berechnen oder speichern.
    #end if:
    i_rr := i_rr + 1: # Gehe zeilenweise durch den unteren linken Teil der Massenmatrix (inkl. Diagonale)
    for k to RowDimension(paramMinRed) do # Spaltenindex der Regressormatrix
      MMregRed[i_rr, k] := diff(MregRed[i, k], xEDD_s_dummy[j, 1]):
      MMreg_qa[i_rr, k] := diff(Mreg_qa[i, k], xEDD_s_dummy[j, 1]):
    end do:
  end do:
end do:
# Dynamik in Plattform-Koordinaten (oben berechnet)
tau_x := ARed:
MMreg_x := MMregRed:
Creg_x := CregRed:
greg_x := gregRed:
# PKM-Dynamik-Funktionen mit MPV bereits eingesetzt
# Erzeuge MPV-Vektor
nMDP := RowDimension(paramMinRed):
PV := Matrix(nMDP,1):
for i from 1 to nMDP do
  PV(i,1) := parse(sprintf("MDP%d%", i)):
end do:

tau_x_mdp := tau_x.PV:
MMreg_x_mdp := MMreg_x.PV:
Creg_x_mdp := Creg_x.PV:
greg_x_mdp := greg_x.PV:

tau_qa_mdp := tau_qa.PV:
MMreg_qa_mdp := MMreg_qa.PV:
Creg_qa_mdp := Creg_qa.PV:
greg_qa_mdp := greg_qa.PV:

# Export
# Maple-Export (zur eventuellen späteren Verarbeitung in Maple)

save tau_x,   sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_reg_maple.m",     robot_name):
save MMreg_x, sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_MMreg_maple.m",   robot_name):
save Creg_x,  sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_tauCreg_maple.m", robot_name):
save greg_x,  sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_taugreg_maple.m", robot_name):

# Matlab Export: Floating base (TODO: Was heißt das? Hier ist doch kein Floating Base?)
# Berechnung der Basis-Belastung ist für manche Basis-Darstellungen falsch (siehe oben unter Gravitationslast). (TODO: Ist das noch aktuell?)

if codeexport_invdyn then
  printf("%s. Beginne Code-Export Inverse Dynamik (Regressor) in Plattform-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  MatlabExport(tau_x, sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_reg_matlab.m", robot_name), codegen_opt):
  printf("%s. Beginne Code-Export Massenmatrix (MDP eingesetzt) in Plattform-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  MatlabExport(MMreg_x, sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_MMreg_matlab.m", robot_name), codegen_opt):
  printf("%s. Beginne Code-Export Coriolis-Vektor (MDP eingesetzt) in Plattform-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  MatlabExport(Creg_x, sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_tauCreg_matlab.m", robot_name), codegen_opt):
  printf("%s. Beginne Code-Export Gravitations-Vektor (MDP eingesetzt) in Plattform-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  MatlabExport(greg_x, sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_taugreg_matlab.m", robot_name), codegen_opt):
end if:

if codeexport_invdyn and RowDimension(Jinv) < 5 and codeexport_actcoord then
  printf("%s. Beginne Code-Export Inverse Dynamik (Regressor) in Antriebs-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  MatlabExport(tau_qa, sprintf("../codeexport/%s/tmp/invdyn_para_actcoord_reg_matlab.m", robot_name), codegen_opt):
  printf("%s. Beginne Code-Export Massenmatrix (Regressor) in Antriebs-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  MatlabExport(MMreg_qa, sprintf("../codeexport/%s/tmp/invdyn_para_actcoord_MMreg_matlab.m", robot_name), codegen_opt):
  printf("%s. Beginne Code-Export Coriolis-Vektor (Regressor) in Antriebs-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  MatlabExport(Creg_qa, sprintf("../codeexport/%s/tmp/invdyn_para_actcoord_tauCreg_matlab.m", robot_name), codegen_opt):
  printf("%s. Beginne Code-Export Gravitations-Vektor (Regressor) in Antriebs-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  MatlabExport(greg_qa, sprintf("../codeexport/%s/tmp/invdyn_para_actcoord_taugreg_matlab.m", robot_name), codegen_opt):
end if:

# Maple-Export

save paramMinRed, sprintf("../codeexport/%s/tmp/minimal_parameter_parrob_maple.m", robot_name):
save RowParamMin, sprintf("../codeexport/%s/tmp/RowMinPar_parallel_maple.m",       robot_name):

MatlabExport(paramMinRed, sprintf("../codeexport/%s/tmp/minimal_parameter_parrob_matlab.m", robot_name), codegen_opt):
MatlabExport(RowParamMin, sprintf("../codeexport/%s/tmp/RowMinPar_parallel.m", robot_name), 2);

# PKM-Dynamik-Funktionen mit MPV bereits eingesetzt

# Maple-Export

save tau_x_mdp,   sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_reg_mdp_maple.m",     robot_name):
save MMreg_x_mdp, sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_MMreg_mdp_maple.m",   robot_name):
save Creg_x_mdp,  sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_tauCreg_mdp_maple.m", robot_name):
save greg_x_mdp,  sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_taugreg_mdp_maple.m", robot_name):

# Matlab Export: 
if codeexport_invdyn then
  printf("%s. Beginne Code-Export Inverse Dynamik (MDP eingesetzt) in Plattform-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  MatlabExport(tau_x_mdp, sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_reg_mdp_matlab.m", robot_name), codegen_opt):
  printf("%s. Beginne Code-Export Massenmatrix (MDP eingesetzt) in Plattform-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  MatlabExport(MMreg_x_mdp, sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_MMreg_mdp_matlab.m", robot_name), codegen_opt):
  printf("%s. Beginne Code-Export Coriolis-Vektor (MDP eingesetzt) in Plattform-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  MatlabExport(Creg_x_mdp, sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_tauCreg_mdp_matlab.m", robot_name), codegen_opt):
  printf("%s. Beginne Code-Export Gravitations-Vektor (MDP eingesetzt) in Plattform-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  MatlabExport(greg_x_mdp, sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_taugreg_mdp_matlab.m", robot_name), codegen_opt):
end if:

if codeexport_invdyn and RowDimension(Jinv) < 5 and codeexport_actcoord then
  printf("%s. Beginne Code-Export Inverse Dynamik (MDP eingesetzt) in Antriebs-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  MatlabExport(tau_qa_mdp, sprintf("../codeexport/%s/tmp/invdyn_para_actcoord_reg_mdp_matlab.m", robot_name), codegen_opt):
  printf("%s. Beginne Code-Export Massenmatrix (MDP eingesetzt) in Antriebs-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  MatlabExport(MMreg_qa_mdp, sprintf("../codeexport/%s/tmp/invdyn_para_actcoord_MMreg_mdp_matlab.m", robot_name), codegen_opt):
  printf("%s. Beginne Code-Export Coriolis-Vektor (MDP eingesetzt) in Antriebs-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  MatlabExport(Creg_qa_mdp, sprintf("../codeexport/%s/tmp/invdyn_para_actcoord_tauCreg_mdp_matlab.m", robot_name), codegen_opt):
  printf("%s. Beginne Code-Export Gravitations-Vektor (MDP eingesetzt) in Antriebs-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  MatlabExport(greg_qa_mdp, sprintf("../codeexport/%s/tmp/invdyn_para_actcoord_taugreg_mdp_matlab.m", robot_name), codegen_opt):
end if:

