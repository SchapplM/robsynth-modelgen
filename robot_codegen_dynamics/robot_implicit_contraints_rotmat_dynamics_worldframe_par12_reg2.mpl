
# Inverse Dynamics for Robot with IC
# Einleitung
# Berechnung der inversen Dynamik für Roboter mit impliziten Zwangsbedingungen
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# implicit_constraints -> Berechnung für Roboter mit impliziten Zwangsbedingungen
# rotmat -> Kinematik wird mit Rotationsmatrizen berechnet
# dynamics -> Berechnung der Dynamik
# worldframe -> Berechnung basierend auf Energien aus Welt-KS (KS W)
# par1 -> Parametersatz 1 (Schwerpunkt als Parameter: SX,SY,SZ) 
# reg2 -> Regressorform
# Autor
# Tim Job (HiWi-Job bei Moritz Schappler), 2019-05
# Moritz Schappler, moritz.schappler@imes.uni-hannover.de
# (C) Institut für Mechatronische Systeme, Universität Hannover
# Sources
# [ParkChoPlo1999] Park, FC and Choi, Jihyeon and Ploen, SR: Symbolic formulation of closed chain dynamics in independent coordinates
# [Docquier2013] Docquier, Nicolas and Poncelet, Antoine and Fisette, Paul: ROBOTRAN: a powerful symbolic gnerator of multibody models (2013)
# [DoThanhKotHeiOrt2009b] Do Thanh et al.: On the inverse dynamics problem of general parallel robots (2009)
# Initialization
interface(warnlevel=0): # Unterdrücke die folgende Warnung.
restart: # Gibt eine Warnung, wenn über Terminal-Maple mit read gestartet wird.
interface(warnlevel=3):
with(LinearAlgebra):
with(ArrayTools):
with(codegen):
with(CodeGeneration):
with(StringTools):
# Einstellungen für Code-Export: Optimierungsgrad (2=höchster) und Aktivierung jedes Terms.
#codegen_act := true: # noch nicht implementiert
codegen_debug := false:
codegen_opt := 2:
read "../helper/proc_MatlabExport":
read "../robot_codegen_definitions/robot_env_IC":
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name_OL):
read "../robot_codegen_definitions/robot_env_IC":
if base_method_name="twist" then # Basis-Methode "twist" wird (hier) nur für fixed Base benutzt
  expstring:="fixb":
elif base_method_name="eulxyz" then 
  expstring:="floatb_eulxyz":
else
  printf("Nicht behandelte Basis-Methode: %s\n", base_method_name):
fi:
# Kennung des Parametersatzes, für den die Dynamikfunktionen erstellt werden sollen. Muss im Repo und in der mpl-Datei auf 1 gelassen werden, da die folgende Zeile mit einem Skript verarbeitet wird.
codegen_dynpar := 1:
regressor_modus := "regressor":
herleitungsverfahren := "lagrange":
codeexport_invdyn := false:
codeexport_grav_inertia := false:
codeexport_coriolisvec := false:
codeexport_regressor := false:
if herleitungsverfahren = "lagrange" then #Dynamik aus Lagrange
# Ergebnisse der inversen Dynamik für par1/2 laden
  if codeexport_invdyn then
    read sprintf("../codeexport/%s/tmp/invdyn_fixb_par%d_maple.m", robot_name_OL, codegen_dynpar):
    tau := Matrix(taus_fixb(7..NQ,1)):
    printf("Generiere inverse Dynamik (%s) für %s basierend auf IC-Jacobi und OL-Dynamik mit Parametersatz %d  und %s\n", herleitungsverfahren, robot_name, codegen_dynpar, base_method_name):
  fi:
# Ergebnisse der inversen Dynamik in Regressorform laden
  if codeexport_regressor then
    read sprintf("../codeexport/%s/tmp/invdyn_%s_%s_maple.m", robot_name_OL, expstring, regressor_modus):
    tau_regressor_s := tau_regressor_s(7..NQ,..):
    printf("Generiere Dynamik in Regressorform (%s) für %s basierend auf IC-Jacobi und OL-Dynamik\n", herleitungsverfahren, robot_name):
  fi:
# Ergebnisse der Massenmatrix und des Gravitationsvektors laden
  if codeexport_grav_inertia then
    read sprintf("../codeexport/%s/tmp/gravload_par%d_maple.m", robot_name_OL, codegen_dynpar):
    taug := Matrix(taug_s(7..NQ,..)):
    read sprintf("../codeexport/%s/tmp/inertia_par%d_maple.m", robot_name_OL, codegen_dynpar):
    MM := Matrix(MM_s(7..NQ,7..NQ)):
    read sprintf("../codeexport/%s/tmp/coriolisvec_par%d_maple.m", robot_name_OL, codegen_dynpar):
    tauCC := Matrix(tauCC_s(7..NQ,..)):
    printf("Generiere Gravitationsvektor und Massenmatrix (%s) für %s basierend auf IC-Jacobi und OL-Dynamik mit Parametersatz %d und %s\n", herleitungsverfahren, robot_name, codegen_dynpar, base_method_name):
  fi:

# Ergebnisse des Coriolisvektors laden. Noch nicht implementiert.
  if codeexport_coriolisvec then
    read sprintf("../codeexport/%s/tmp/coriolisvec_par%d_maple.m", robot_name_OL, codegen_dynpar):
    tauCC := Matrix(tauCC_s(7..NQ,..)):
    read sprintf("../codeexport/%s/tmp/inertia_par%d_maple.m", robot_name_OL, codegen_dynpar):
    MM := Matrix(MM_s(7..NQ,7..NQ)):
    printf("Generiere Coriolisvektor (%s) für %s basierend auf IC-Jacobi und OL-Dynamik mit Parametersatz %d und %s\n", herleitungsverfahren, robot_name, codegen_dynpar, base_method_name):
  fi:
else #Dynamik aus Newton-Euler
# Ergebnisse der inversen Dynamik für par1/2 laden
  read sprintf("../codeexport/%s/tmp/invdyn_twist_NewtonEuler_linkframe_par%d_maple.m", robot_name_OL, codegen_dynpar):
  tau := Matrix(tau_J(1..NQJ,..)):
  taug := copy(tau):
  for i from 1 to NQ do
    taug := subs({qD_s[i,1]=0},taug):
    taug := subs({qDD_s[i,1]=0},taug):
  end do:
  printf("Generiere Gravitationsvektor und gesamte inverse Dynamik (%s) für %s basierend auf IC-Jacobi und OL-Dynamik mit Parametersatz %d\n", herleitungsverfahren, robot_name, codegen_dynpar):
fi:
# Ergebnisse der impliziten Zwangsbedingungen laden
read sprintf("../codeexport/%s/tmp/kinconstr_impl_projection_jacobian_maple", robot_name):
B21 := B21:
read sprintf("../codeexport/%s/tmp/kinconstr_impl_projection_jacobian_derivative_maple", robot_name):
B21D := B21D:
# Ergebnisse des Positionsvektors und der aktiven und passiven Gelenke laden
read sprintf("../codeexport/%s/tmp/positionVector_NQJ_maple.m", robot_name):
posNQJ := posNQJ:
# Pertubationsmatrizen berechnen
# Entferne aktive/passive Koordinaten zur Umrechnung
# Berechne die Anzahl aller aktiven und passiven Gelenke (inkl virtueller Schnittgelenke)
NAJ := add(mu(k), k=1..NJ):
NPJ := NJ-NAJ:
# Bestimme Indizes der aktiven und passiven Gelenke
# Entspricht Partitionierung der Gelenkwinkel in q1 und q2 in [Docquier2013]. Hier kann die Reihenfolge von aktiven und passiven Winkeln auch durchmischt sein.
IndAct := Matrix(NAJ, 1):
IndPass:= Matrix(NPJ, 1):
ka := 1: kp := 1:
for i from 1 to NJ do
  if mu(i) = 1 then
    IndAct(ka) := i:
    ka := ka + 1:
  else
    IndPass(kp) := i:
    kp := kp + 1:
  end if:
end do:
# Entfernt die Einträge aus IndPass/IndAct, die virtuelle Schnittglenke sind
for i from 1 to NJ do
  if posNQJ(i) = 0 then
    posAct := ListTools[Search](i, convert(IndAct, list)):
    if posAct <> 0 then
      IndAct := DeleteRow(IndAct,posAct):
    end if:
    #posPass := ListTools[Search](i, convert(IndPass, list)):
    #if posPass <> 0 then
    #  IndPass := DeleteRow(IndPass,posPass):
    #end if:
  end if:
end do:
NQJA := RowDimension(IndAct):
NQJP := NQJ-NQJA: #RowDimension(IndPass):
IndPass:= Matrix(NQJP, 1):
kp := 1: ka := 1:
for i from 1 to NQJ do
  if ListTools[Search](i, convert(IndAct, list)) = 0 then
    IndPass(kp) := i:
    kp := kp + 1:
  end if:
end do:
# Permutationsmatrizen berechnen zur Umrechnung von aufgeteilten Koordinaten
# (aktiv/passiv nacheinander) in Koordinaten der Baumstruktur (aktiv/passiv gemischt) 
P1 := Matrix(NQJA,NQJ):
P2 := Matrix(NQJP,NQJ):
for i from 1 to NQJA do
  P1(i,IndAct(i)) := 1:
end do:
for i from 1 to NQJP do
  P2(i,IndPass(i)) := 1:
end do:
# Inverse Dynamik
# explizit [Docquier2013], Gl. 12
W := Transpose(P1 + Transpose(B21).P2):
WD := Transpose(Transpose(B21D).P2):
#q1D_s := Transpose(W).qJD_s:
tauIC := Transpose(W).tau:
taugIC := Transpose(W).taug:
# [DoThanhKotHeiOrt2009b], Gl. (23); [ParkChoPlo1999] Gl. (56)
#tauCCIC := Transpose(W).tauCC + Transpose(W).MM.WD.q1D_s: # noch nicht implementiert
;
tauIC_regressor := P1.tau_regressor_s + Transpose(B21).P2.tau_regressor_s:
if herleitungsverfahren = "newton" then
  tau_MM := copy(tau):
  for i to NQ do
    tau_MM := subs(qD_s[i, 1] = 0,tau_MM);
  end do:

  MM := Matrix(NQJ, NQJ):
  for i to NQJ do 
    for j to NQJ do 
      MM[i, j] := diff(tau_MM[i, 1], qDD_s[6+j, 1]):
    end do:
  end do:
end if:
# [ParkChoPlo1999], Gl. 55; [DoThanhKotHeiOrt2009b], Gl. (23)
MMIC := Transpose(W).MM.W:
# Export
# Export der Belastung der Gelenke
# Floating Base
if codeexport_invdyn and not(base_method_name="twist") then
  MatlabExport(tauIC, sprintf("../codeexport/%s/tmp/invdyn_floatb_%s_par%d_ic_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
if codeexport_regressor and not(base_method_name="twist") then
  MatlabExport(tauIC_regressor, sprintf("../codeexport/%s/tmp/invdyn_floatb_%s_%s_ic_matlab.m", robot_name, base_method_name, regressor_modus), codegen_opt):
end if:
# Fixed Base
if codeexport_invdyn then
  tauIC_fixb:=tauIC:
  for i from 1 to NQB do
    tauIC_fixb := subs({X_base_s[i,1]=0},tauIC_fixb):
  end do:
  for i from 1 to 6 do
    tauIC_fixb := subs({V_base_s[i,1]=0},tauIC_fixb):
    tauIC_fixb := subs({VD_base_s[i,1]=0},tauIC_fixb):
  end do:
  MatlabExport(tauIC_fixb, sprintf("../codeexport/%s/tmp/invdyn_fixb_par%d_ic_matlab.m", robot_name, codegen_dynpar), codegen_opt):
end if:
if codeexport_grav_inertia then
  taugIC_fixb:=taugIC:
  MMIC_fixb:=MMIC:
  for i from 1 to NQB do
    MMIC_fixb := subs({X_base_s[i,1]=0},MMIC_fixb):
  end do:
  for i from 1 to 6 do
    MMIC_fixb := subs({V_base_s[i,1]=0},MMIC_fixb):
    MMIC_fixb := subs({VD_base_s[i,1]=0},MMIC_fixb):
  end do:
  MatlabExport(taugIC_fixb, sprintf("../codeexport/%s/tmp/gravload_joint_floatb_twist_par%d_ic_matlab.m", robot_name, codegen_dynpar), codegen_opt):
  MatlabExport(MMIC_fixb, sprintf("../codeexport/%s/tmp/inertia_joint_joint_floatb_%s_par%d_ic_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
# Noch nicht implementiert
if codeexport_coriolisvec then
  tauCCIC_fixb:=tauCCIC:
  for i from 1 to NQB do
    tauCCIC_fixb := subs({X_base_s[i,1]=0},tauCCIC_fixb):
  end do:
  for i from 1 to 6 do
    tauCCIC_fixb := subs({V_base_s[i,1]=0},tauCCIC_fixb):
    tauCCIC_fixb := subs({VD_base_s[i,1]=0},tauCCIC_fixb):
  end do:
  MatlabExport(tauCCIC_fixb, sprintf("../codeexport/%s/tmp/coriolisvec_joint_fixb_twist_par%d_ic_matlab.m", robot_name, codegen_dynpar), codegen_opt):
end if:
if codeexport_regressor then
  tauIC_regressor_fixb:=tauIC_regressor:
  for i from 1 to NQB do
    tauIC_regressor_fixb := subs({X_base_s[i,1]=0},tauIC_regressor_fixb):
  end do:
  for i from 1 to 6 do
    tauIC_regressor_fixb := subs({V_base_s[i,1]=0},tauIC_regressor_fixb):
    tauIC_regressor_fixb := subs({VD_base_s[i,1]=0},tauIC_regressor_fixb):
  end do:
  MatlabExport(tauIC_regressor_fixb, sprintf("../codeexport/%s/tmp/invdyn_fixb_%s_ic_matlab.m", robot_name, regressor_modus), codegen_opt):
end if:
if herleitungsverfahren = "newton" then
  tauIC_fixb:=tauIC:
  taugIC_fixb:=taugIC:
  MMIC_fixb:=MMIC:
  for i from 1 to NQB do
    tauIC_fixb := subs({X_base_s[i,1]=0},tauIC_fixb):
    MMIC_fixb := subs({X_base_s[i,1]=0},MMIC_fixb):
  end do:
  for i from 1 to 6 do
    tauIC_fixb := subs({V_base_s[i,1]=0},tauIC_fixb):
    tauIC_fixb := subs({VD_base_s[i,1]=0},tauIC_fixb):
    MMIC_fixb := subs({V_base_s[i,1]=0},MMIC_fixb):
    MMIC_fixb := subs({VD_base_s[i,1]=0},MMIC_fixb):
  end do:
  MatlabExport(tauIC_fixb, sprintf("../codeexport/%s/tmp/invdyn_fixb_snew_par%d_ic_matlab.m", robot_name, codegen_dynpar), codegen_opt):
  MatlabExport(taugIC_fixb, sprintf("../codeexport/%s/tmp/gravload_joint_floatb_twist_snew_par%d_ic_matlab.m", robot_name, codegen_dynpar), codegen_opt):
  MatlabExport(MMIC_fixb, sprintf("../codeexport/%s/tmp/inertia_joint_joint_floatb_%s_snew_par%d_ic_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
# Export Anzahl der aktiven Koordinaten NAJ
MatlabExport(NAJ, sprintf("../codeexport/%s/tmp/NAJ_ic_matlab.m", robot_name), codegen_opt):

