
# Berechnung und Projektion der Dynamikgleichungen
# Einleitung
# Berechnung und Projektion der Dynamikgleichungen
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# para -> Berechnung für einen parallelen Roboter
# rotmat -> Kinematik wird mit Rotationsmatrizen berechnet
# projection -> Die Dynamikgleichungen werden auf EE-Koordinaten projiziert
# dynamics -> Berechnung der Dynamik
# Autor
# Tim Job (Studienarbeit bei Moritz Schappler), 2018-12
# Moritz Schappler, moritz.schappler@imes.uni-hannover.de
# (C) Institut für Mechatronische Systeme, Universität Hannover
# Sources
# [Abdellatif2007] Modellierung, Identifikation und robuste Regelung von Robotern mit parallelkinematischen Strukturen
# [Job2018_S759] Job, T. (Studienarbeit; Betreuer Moritz Schappler): Implementierung einer strukturunabhängigen Dynamikmodellierung für parallelkinematische Maschinen (2018)
# Initialization
interface(warnlevel=0): # Unterdrücke die folgende Warnung.
restart: # Gibt eine Warnung, wenn über Terminal-Maple mit read gestartet wird.
interface(warnlevel=3):
with(LinearAlgebra):
with(codegen):
with(CodeGeneration):
with(StringTools): # Für Zeitausgabe
;
# Einstellungen für Code-Export: Optimierungsgrad (2=höchster).
#codegen_act := true: # noch nicht implementiert
codegen_debug := false:
codegen_opt := 2:
codeexport_actcoord := false: # Generierung der Dynamik in Antriebskoordinaten nicht standardmäßig (hoher Rechenaufwand)
;
read "../helper/proc_MatlabExport":
read "../helper/proc_simplify2":
read "../robot_codegen_definitions/robot_env_par":
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", leg_name):
# Kennung des Parametersatzes, für den die Dynamikfunktionen erstellt werden sollen. Muss im Repo und in der mpl-Datei auf 1 gelassen werden, da die folgende Zeile mit einem Skript verarbeitet wird.
codegen_dynpar := 1:
# Term-Vereinfachungen konfigurieren
if not assigned(simplify_options) or simplify_options(9)=-1 then # Standard-Einstellungen:
  use_simplify := 1: # standardmäßig simplify-Befehle anwenden
else # Benutzer-Einstellungen:
  use_simplify := simplify_options(9): # neunter Eintrag ist für Dynamik-Regressor
end if:
# Ergebnisse der zusätzlichen Definitionen für parallele Roboter laden
read "../robot_codegen_definitions/robot_env_par":
read sprintf("../codeexport/%s/tmp/para_definitions", robot_name):
# Ergebnisse der Plattform-Dynamik laden (aus robot_para_plattform_rotmat_dynamics.mw)
read "../robot_codegen_definitions/robot_env_par":
dynamicsfile := sprintf("../codeexport/%s/tmp/floatb_platform_dynamic_maple.m", robot_name):
if FileTools[Exists](dynamicsfile) then
  read dynamicsfile:
else
  printf("%s. PKM-Dynamik konnte nicht geladen werden. Abbruch der Berechnung.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  quit: # Funktioniert in GUI nicht richtig...
  robot_name := "": # ...Daher auch Löschung des Roboternamens.
end if:
# Neu-Definition der geladenen Variablen
MME:=MME:
cvecE:=cvecE:
gE:=gE:
tauE:=tauE:
H:=H:
dH:=dH:
if use_simplify >= 1 then
  tmp_t1:=time():
  tmp_l1 := length(MME):
  printf("%s. Beginne Vereinfachung: Plattform-Massenmatrix. Länge: %d.\n", \
    FormatTime("%Y-%m-%d %H:%M:%S"), tmp_l1):
  MME := simplify2(MME):
  tmp_t2:=time():
  tmp_l2 := length(MME):
  printf("%s. Plattform-Massenmatrix vereinfacht. Länge: %d->%d. Rechenzeit %1.1fs.\n", \
    FormatTime("%Y-%m-%d %H:%M:%S"), tmp_l1, tmp_l2, tmp_t2-tmp_t1):
    
  tmp_t1:=time():
  tmp_l1 := length(gE):
  printf("%s. Beginne Vereinfachung: Plattform-Gravitationsterm. Länge: %d.\n", \
    FormatTime("%Y-%m-%d %H:%M:%S"), tmp_l1):
  gE := simplify2(gE):
  tmp_t2:=time():
  tmp_l2 := length(gE):
  printf("%s. Plattform-Gravitationsterm vereinfacht. Länge: %d->%d. Rechenzeit %1.1fs.\n", \
    FormatTime("%Y-%m-%d %H:%M:%S"), tmp_l1, tmp_l2, tmp_t2-tmp_t1):
end if:

# Ergebnisse der Kinematik für parallelen Roboter laden
read "../robot_codegen_definitions/robot_env_par":
kinematicsfile := sprintf("../codeexport/%s/tmp/kinematics_%s_platform_maple.m", robot_name, base_method_name):
if FileTools[Exists](kinematicsfile) then
  read kinematicsfile:
else
  printf("%s. PKM-Kinematik konnte nicht geladen werden. Abbruch der Berechnung.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  quit: # Funktioniert in GUI nicht richtig...
  robot_name := "": # ...Daher auch Löschung des Roboternamens.
end if:
read "../robot_codegen_definitions/robot_env_par": # Nochmal laden, um Standard-Einstellungen überschreiben zu können.
# Neu-Definition der von dieser Datei gelesenen Variablen, damit sie im Workspace erscheinen
pivotMat := pivotMat:
pivotMatMas := pivotMatMas:
Jinv := Jinv:
JBinv_i := JBinv_i:
U_i := U_i:

# Lade "robotics_repo_path"-File mit Link zum "imes-robotics-matlab"-Repo
read("../robotics_repo_path"):
# Lade die Funktionen aus dem "imes-robotics-matlab"-Repo
read(sprintf("%s/transformation/maple/proc_eul%s2r", robotics_repo_path, angleConvLeg)):
read(sprintf("%s/transformation/maple/proc_eul%sjac", robotics_repo_path, "zyx")): # TODO: Muss hier die Winkelkonvention eingesetzt werden? Wird das hier gebraucht?
# TODO: Euler-Funktion mit "parse"-Befehl hier definieren
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
# Startzeit messen zur Beurteilung der Zeitdauer einzelner Schritte
st := time():
# Physikalische Parameter der durch Koppelgelenke bewegten Körper zu Null setzen.
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
# Ergebnisse G-Vektor der Beinkette  laden.
# Die Rotation der Basis wird nur in der Jacobi-Matrix der inverse Kinematik berücksichtigt. Deshalb muss der Gravitationsvektor ebenfalls an die Rotation angepasst werden.
g1 := gtmp1:
g2 := gtmp2:
g3 := gtmp3:

dynamicsfile_leg := sprintf("../codeexport/%s/tmp/gravload_par%d_maple.m", leg_name, codegen_dynpar):
if FileTools[Exists](dynamicsfile_leg) then
  read dynamicsfile_leg:
else
  printf("%s. Beinketten-Dynamik (g) konnte nicht geladen werden. Abbruch der Berechnung.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  quit: # Funktioniert in GUI nicht richtig...
  robot_name := "": # ...Daher auch Löschung des Roboternamens.
end if:
G := simplify2(Matrix(taug_s(7..NQ,1))):
if use_simplify >= 1 then
  tmp_t1:=time():
  tmp_l1 := length(G):
  printf("%s. Beginne Vereinfachung: Beinketten-Gravitationsterme. Länge: %d.\n", \
    FormatTime("%Y-%m-%d %H:%M:%S"), tmp_l1):
  G := simplify2(G):
  tmp_t2:=time():
  tmp_l2 := length(G):
  printf("%s. Beinketten-Gravitationsterme vereinfacht. Länge: %d->%d. Rechenzeit %1.1fs.\n", \
    FormatTime("%Y-%m-%d %H:%M:%S"), tmp_l1, tmp_l2, tmp_t2-tmp_t1):
end if:
unassign('g1','g2','g3'):
g := <g1;g2;g3>:
Rmat := Transpose(parse(sprintf("eul%s2r",angleConvLeg))(frame_A_i(1..3,1))):
gtmp1 := (Rmat.g)(1):
gtmp2 := (Rmat.g)(2):
gtmp3 := (Rmat.g)(3):
G := G:

# Ergebnisse C-Vektor der Beinkette laden
dynamicsfile_leg := sprintf("../codeexport/%s/tmp/coriolisvec_par%d_maple.m", leg_name, codegen_dynpar):
if FileTools[Exists](dynamicsfile_leg) then
  read dynamicsfile_leg:
else
  printf("%s. Beinketten-Dynamik (c) konnte nicht geladen werden. Abbruch der Berechnung.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  quit: # Funktioniert in GUI nicht richtig...
  robot_name := "": # ...Daher auch Löschung des Roboternamens.
end if:
Cvec := simplify2(Matrix(tauCC_s(7..NQ,1))):
if use_simplify >= 1 then
  tmp_t1:=time():
  tmp_l1 := length(Cvec):
  printf("%s. Beginne Vereinfachung: Beinketten-Coriolisterme. Länge: %d.\n", \
    FormatTime("%Y-%m-%d %H:%M:%S"), tmp_l1):
  Cvec := simplify2(Cvec):
  tmp_t2:=time():
  tmp_l2 := length(Cvec):
  printf("%s. Beinketten-Coriolisterme vereinfacht. Länge: %d->%d. Rechenzeit %1.1fs.\n", \
    FormatTime("%Y-%m-%d %H:%M:%S"), tmp_l1, tmp_l2, tmp_t2-tmp_t1):
end if:

# Ergebnisse M-Matrix der Beinkette laden
dynamicsfile_leg := sprintf("../codeexport/%s/tmp/inertia_par%d_maple.m", leg_name, codegen_dynpar):
if FileTools[Exists](dynamicsfile_leg) then
  read dynamicsfile_leg:
else
  printf("%s. Beinketten-Dynamik (M) konnte nicht geladen werden. Abbruch der Berechnung.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  quit: # Funktioniert in GUI nicht richtig...
  robot_name := "": # ...Daher auch Löschung des Roboternamens.
end if:
MM := MM_s(7..NQ,7..NQ):
if use_simplify >= 1 then
  tmp_t1:=time():
  tmp_l1 := length(MM):
  printf("%s. Beginne Vereinfachung: Beinketten-Massenmatrix. Länge: %d.\n", \
    FormatTime("%Y-%m-%d %H:%M:%S"), tmp_l1):
  MM := simplify2(MM):
  tmp_t2:=time():
  tmp_l2 := length(MM):
  printf("%s. Beinketten-Massenmatrix vereinfacht. Länge: %d->%d. Rechenzeit %1.1fs.\n", \
    FormatTime("%Y-%m-%d %H:%M:%S"), tmp_l1, tmp_l2, tmp_t2-tmp_t1):
end if:
printf("%s. Alle Daten geladen. Generiere Dynamik für PKM %s mit Parametersatz %d\n", FormatTime("%Y-%m-%d %H:%M:%S"), robot_name, codegen_dynpar, base_method_name):

# Berechne Dynamik-Matrizen für alle Beine
# Dupliziere alle berechneten Matrizen. i steht für den Index des jeweiligen Beines
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
# Berechnung der Kräfte/Momente an den Gelenken der jeweiligen Beine und Projektion auf EE-Plattform
# Abdellatif2007 S.38 (3.27); [Job2018_S759], S. 29

for i to N_LEGS do
  Jtmp := Multiply(Transpose(U_i(..,..,i)),Transpose(JBinv_i(..,..,i))):
  qDtmp := Multiply(JBinv_i(..,..,i),U_i(..,..,i).H.xED_s):
  A||i := Multiply(JBinv_i(..,..,i),JBD_i(..,..,i)):
  if use_simplify >= 1 then
    tmp_t1:=time():
    tmp_l1 := length(A||i):
    printf("%s. Beginne Vereinfachung: Beinketten-Projektionsmatrix A%d. Länge: %d.\n", \
      FormatTime("%Y-%m-%d %H:%M:%S"), i, tmp_l1):
    A||i := simplify2(A||i):
    tmp_t2:=time():
    tmp_l2 := length(A||i):
    printf("%s. Beinketten-Projektionsmatrix A%d vereinfacht. Länge: %d->%d. Rechenzeit %1.1fs.\n", \
      FormatTime("%Y-%m-%d %H:%M:%S"), i, tmp_l1, tmp_l2, tmp_t2-tmp_t1):
  end if:
  B||i := Multiply(-MM||i,Multiply(A||i,qDtmp)):

  # [Job2018_S759], Term in der Summe in Gl. (3.50)
  MMs||i := Jtmp . MM||i . Transpose(Jtmp) . H:
  # [Job2018_S759], Term in der Summe in Gl. (3.51)
  cvecs||i := Jtmp.MM||i.JBinv_i(..,..,i).(U_i(..,..,i).dH + UD_i(..,..,i).H).xED_s + Jtmp.B||i + Jtmp.Cvec||i:
  # [Job2018_S759], Term in der Summe in Gl. (3.52)
  gvecs||i := Jtmp.G||i:
  
  tau||i := Jtmp.MM||i.JBinv_i(..,..,i).(U_i(..,..,i).H.xEDD_s+U_i(..,..,i).dH.xED_s+UD_i(..,..,i).H.xED_s) + Multiply(Jtmp,(B||i+Cvec||i+G||i)):

  taus||i := MMs||i.xEDD_s + cvecs||i + gvecs||i:
end do:

# Abdellatif2007 S.40 (3.33); [Job2018_S759], (3.49)
# Aufsummieren aller Kräfte, projiziert auf EE-Plattform
Tmp := 0:
for i to N_LEGS do
  Tmp := Tmp + tau||i:
end do:
# Addiere Inverse Dynamik der Plattform
tauGes := Tmp + tauE:
# Aufsummieren aller Massenmatrizen, projiziert auf EE-Plattform
# [Job2018_S759], (3.50)
Tmp := 0:
for i to N_LEGS do
  Tmp := Tmp + MMs||i:
end do:
# Addiere Massenmatrix der Plattform
MMGes := Tmp + MME:
# Aufsummieren aller Coriolisvektoren, projiziert auf EE-Plattform
# [Job2018_S759], (3.51)
Tmp := 0:
for i to N_LEGS do
  Tmp := Tmp + cvecs||i:
end do:
# Addiere Coriolisvektor der Plattform
cvecGes := Tmp + cvecE:
# Aufsummieren aller Gravitiationsvektoren, projiziert auf EE-Plattform
# [Job2018_S759], (3.52)
Tmp := 0:
for i to N_LEGS do
  Tmp := Tmp + gvecs||i:
end do:
# Addiere Gravitiationsvektor der Plattform
gGes := Tmp - gE:
#tauGes := MMGes.xEDD_s + cvecGes + gGes:
# Replace Joint Velocities
# Substituiere die Gelenkgeschwindigkeiten über H-, Ui- und JBi-Matrix mit EE-Geschwindikeiten
Tmp := 0:
for i to N_LEGS do
  Tmp := Multiply(H,xED_s):
  Tmp := Multiply(U_i(..,..,i),Tmp):
  z||i := Multiply(JBinv_i(..,..,i),Tmp):
end do:
for i to 6 do
  for j to N_LEGS do
    for l to NQJ_parallel do
      tauGes(i,1)  := subs({qJD_i_s(l,j)=z||j(l)},tauGes(i,1)):
      cvecGes(i,1) := subs({qJD_i_s(l,j)=z||j(l)},cvecGes(i,1)):
      gGes(i,1)    := subs({qJD_i_s(l,j)=z||j(l)},gGes(i,1)):
      for k to 6 do
        MMGes(i,k) := subs({qJD_i_s(l,j)=z||j(l)},MMGes(i,k)):
      end do:
    end do:
  end do:
end do:
# Export
# Wähle die Einträge aus Dynamikgleichungen, die für Freiheitsgrade des Roboters relevant sind.
# (über die Auswahl-Matrix "pivotMat").
#Jtestinv := Matrix(6,6,symbol=Jentry):
#Jtest := MatrixInverse(Jtestinv):
#Jtest := simplify(Jtest):
#for i to RowDimension(Jtest) do
#  for j to ColumnDimension(Jtest) do
#    for k to RowDimension(Jtest) do
#      for l to ColumnDimension(Jtest) do
#        Jtest(i,j) := subs(Jentry[k,l]=simplify(Jinv(k,l)),Jtest(i,j)):
#      end do:
#    end do:
#  end do:
#end do:
# Dynamik-Terme in Plattform-Koordinaten
tau_x     := pivotMat.tauGes:
MMGes_x   := pivotMat.MMGes.Transpose(pivotMatMas):
cvecGes_x := pivotMat.cvecGes:
gGes_x    := pivotMat.gGes:
# Maple-Export (zum späteren Code-Export)
save tau_x,     sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_par%d_maple.m",      robot_name, codegen_dynpar):
save MMGes_x,   sprintf("../codeexport/%s/tmp/inertia_para_plfcoord_par%d_maple.m",     robot_name, codegen_dynpar):
save cvecGes_x, sprintf("../codeexport/%s/tmp/coriolisvec_para_plfcoord_par%d_maple.m", robot_name, codegen_dynpar):
save gGes_x,    sprintf("../codeexport/%s/tmp/gravvec_para_plfcoord_par%d_maple.m",     robot_name, codegen_dynpar):
# Dynamik in Antriebs-Koordinaten umrechnen. Nur machen, wenn die Jacobi-Matrix einfach genug ist. Sonst ist die symbolische Invertierung zu teuer und sollte numerisch gemacht werden
# [Job2018_S759], S. 30; Gl. 3.53, 3.54
if RowDimension(Jinv) < 5 and codeexport_actcoord then
  printf("%s. Beginn der Matrix-Invertierung.  CPU-Zeit bis hier: %1.2fs.\n", FormatTime("%Y-%m-%d %H:%M:%S"), time()-st):
  J:=MatrixInverse(Jinv): # TODO: Matrix-Invertierung in eigenem Skript (bei der Kinematik; dort mit Platzhalter-Matrix invertieren)
  save J, sprintf("../codeexport/%s/tmp/jacobian_maple.m", robot_name): # TODO: Besseren Namen wählen und dies im Kinematik-Skript machen.
  printf("%s. Matrix-Invertierung beendet. CPU-Zeit bis hier: %1.2fs.\n", FormatTime("%Y-%m-%d %H:%M:%S"), time()-st):
  # J:=simplify(J):
  # printf("%s. Optimierung beendet. CPU-Zeit bis hier: %1.2fs.\n", FormatTime("%Y-%m-%d %H:%M:%S"), time()-st):
  tau_qa     := Transpose(J) . tau_x:
  MMGes_qa   := Transpose(J) . MMGes_x:
  cvecGes_qa := Transpose(J) . cvecGes_x:
  gGes_qa    := Transpose(J) . gGes_x:
  # Maple-Export (zur eventuellen späteren Verarbeitung in Maple)
  save tau_qa,     sprintf("../codeexport/%s/tmp/invdyn_para_actcoord_par%d_maple.m",      robot_name, codegen_dynpar):
  save MMGes_qa,   sprintf("../codeexport/%s/tmp/inertia_para_actcoord_par%d_maple.m",     robot_name, codegen_dynpar):
  save cvecGes_qa, sprintf("../codeexport/%s/tmp/coriolisvec_para_actcoord_par%d_maple.m", robot_name, codegen_dynpar):
  save gGes_qa,    sprintf("../codeexport/%s/tmp/gravvec_para_actcoord_par%d_maple.m",     robot_name, codegen_dynpar):
end if:
printf("%s. Speicherung der Dynamik-Terme in symbolischer Form beendet.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
# Der Matlab-Export wird in einem anderen Arbeitsblatt gemacht.
