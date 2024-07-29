
# Definitions for Robot Dynamics Code Generation
# Init
# Erstelle Definitionen für die Maple-Skripte zur Berechnung von Kinematik und Dynamik des Roboters
# 
# Quellen:
# [1] Sousa, C. D. and Cortesao, R.: Physical feasibility of robot base inertial parameter identification: A linear matrix inequality approach (2014)
# [2] Ayusawa, K. and Venture, G. and Nakamura, Y.: Identifiability and identification of inertial parameters using the underactuated base-link dynamics for legged multibody systems (2013)
# [3] Fujimoto, Y. and Obata, S. and Kawamura, A.: Robust biped walking with active interaction control between foot and ground (1998)
# [4] Khalil, W. and Kleinfinger, J.-F.: Minimum operations and minimum parameters of the dynamic models of tree structure robots (1987)
# 
# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
interface(warnlevel=0): # Unterdrücke die folgende Warnung.
restart: # Gibt eine Warnung, wenn über Terminal-Maple mit read gestartet wird.
interface(warnlevel=3):
with(LinearAlgebra):
with(StringTools):
read "../helper/proc_MatlabExport":
read "../helper/proc_convert_t_s":
read "../helper/proc_vec2skew":
codegen_act := true:
# Lese Umgebungsvariable für Codegenerierung.
read "../robot_codegen_definitions/robot_env":
printf("Generiere Parameter für %s\n",robot_name):
# Robotics Definitions
# Joint Angles, Velocities and Accelerations in time depending and substitution form.
# Time Depending form is for time differentiation.
# Substitution Form is for partial differentiation and code export.
qJ_t := Matrix(NQJ, 1):
qJ_s := Matrix(NQJ, 1):
qJD_s := Matrix(NQJ, 1):
qJDD_s := Matrix(NQJ, 1):
for i from 1 to NQJ do
  qJ_t(i,1):=parse(sprintf("qJ%d(t)", i)):
  qJ_s(i,1):=parse(sprintf("qJ%ds", i)):
  qJD_s(i,1):=parse(sprintf("qJD%ds", i)):
  qJDD_s(i,1):=parse(sprintf("qJDD%ds", i)):
end do:
qJD_t := diff~(qJ_t, t):
qJDD_t := diff~(qJD_t, t):
# Gravity vector in world frame
if not assigned(g_world) then
  g_world := Matrix(3, 1):
  g_world(1 .. 3, 1) := <g1, g2, g3>:
end if:
# Position und Orientierung der Basis. Die Orientierung ist mit XYZ-Euler-Winkeln definiert, die aber nicht im weiteren Algorithmus verwendet werden (nur Platzhalter).
# Eine Invertierung der Orientierungsdarstellung sollte nicht notwendig werden, von daher kein Problem mit Orientierungsrepräsentationssingularität.
# gem. [2], S. 4 X_base_t SE(3): Position und Orientierung.
NQB := 6:
X_base_t:=Matrix(6, 1, [rx_base(t),ry_base(t),rz_base(t),alphax_base(t), betay_base(t), gammaz_base(t)]):
X_base_s:=Matrix(6, 1, [rxs_base,rys_base,rzs_base,alphaxs_base, betays_base, gammazs_base]):
# Geschwindigkeit der Basis (pelvis)
# Twist: Verallgemeinerte Geschwindigkeit. Diese Darstellung ist nicht konsistent (Ableitung der Position stimmt nicht mit Ausdruck der Geschwindigkeit überein)
V_base_t:=Matrix(6, 1, [diff~(rx_base(t),t),diff~(ry_base(t),t),diff~(rz_base(t),t),omegax_base(t), omegay_base(t), omegaz_base(t)]):
V_base_s:=Matrix(6, 1, [vxs_base,vys_base,vzs_base,omegaxs_base, omegays_base, omegazs_base]):
# Zeitableitung davon
VD_base_t:=diff~(V_base_t,t):
VD_base_s:=Matrix(6, 1, [vDxs_base,vDys_base,vDzs_base,omegaDxs_base, omegaDys_base, omegaDzs_base]):
# Name der Methode für die Orientierungsdarstellung der Basis
base_method_name := "twist":
# Umrechnung von der Ableitung der Basis-Orientierung zu Winkelgeschwindigkeiten:
# Wird hier nicht weiter beachtet (physikalisch falsch für eine Betrachtung der Dynamik der Basis).
# Ermöglicht eine einfachere Berechnung für Mechanismen, wo die Rückwirkung auf die Basis nicht in der Vorwärtsdynamik simuliert werden muss.
T_basevel := IdentityMatrix(3, 3):
# Verallgemeinerte Koordinaten, gem [2], S. 4, [3], S.1
NQ:=NQJ+NQB:
q_t := Matrix(NQ,1, <X_base_t, qJ_t>):
q_s := Matrix(NQ,1, <X_base_s, qJ_s>):
qD_t:= Matrix(NQ,1, <V_base_t, qJD_t>):
qD_s:= Matrix(NQ,1, <V_base_s, qJD_s>):
qDD_t:= Matrix(NQ,1, <VD_base_t, qJDD_t>):
qDD_s:= Matrix(NQ,1, <VD_base_s, qJDD_s>):
# MDH-Gelenkwinkel neu speichern (Definition der verallg. Koordinaten war dort noch nicht bekannt
theta := value(theta):
# Standard-Werte festlegen
# Anzahl der Körper (Number of Links):
if not assigned(NL) then
  NL := NJ + 1:
  printf("Variable NL ist nicht gegeben. Insgesamt %d Gelenke. Nehme an, dass jedes Gelenk einem Körper zugeordnet ist (keine Schleifen)\n", NJ):
else
  NVJ := NJ - (NL - 1):
  printf("Variable NL=%d ist gegeben. Insgesamt %d Gelenke. Davon sind die ersten %d einem Körper zugeordnet und die letzten %d virtuell.\n", NL, NJ, NJ-NVJ, NVJ):
end if:
# Gelenktyp (0=Revolute, 1=Prismatic, 2=Static). Sollte in der Definition festgelegt sein. Falls nicht, wird alles auf Revolute gesetzt
if not assigned(sigma) then
  sigma := Matrix(NJ,1):
  printf("Variable sigma ist nicht gegeben. Setze alle Gelenke als Drehgelenk (sigma=1)\n"):
end if:
# Aktuierung (1=aktiv, 0=passiv). Sollte in der Definition festgelegt sein. Falls nicht, wird alles auf Aktiv gesetzt
if not assigned(mu) then
  mu := Matrix(NJ,1,1):
  printf("Variable mu ist nicht gegeben. Setze alle Gelenke als aktiv (mu=1)\n"):
end if:
# Segmentreihenfolge der Roboterstruktur. Sollte in der Definition festgelegt sein. Falls nicht, wird eine serielle Kette angenommen
if not assigned(v) then
  v := Matrix(NL-1,1,i->i-1):
  printf("Variable v ist nicht gegeben. Setze serielle Kette (0,1,2,...)\n"):
end if:
# Parameter für Baumstruktur. Sollte in der Definition festgelegt sein. Falls nicht, wird eine serielle Kette angenommen
if not assigned(b) then
  b := Matrix(NJ,1):
  printf("Variable b ist nicht gegeben. zusätzliche Verschiebung auf Null\n"):
end if:
if not assigned(beta) then
  beta := Matrix(NJ,1):
  printf("Variable beta ist nicht gegeben. zusätzliche Verschiebung auf Null\n"):
end if:
# Eingabe prüfen
# Validität der Vorgänger-Indizes. Konvention: Vorgänger hat kleineren Index. Siehe Khalil-Publikationen (dort evtl nur implizit so verwendet).
for i from 1 to NJ do
  if v(i) >= i then
    printf("v(%d)=%d. Verletzt die Regel, dass die Vorgänger-Nummer kleiner sein muss als die aktuelle Nummer.\n", i, v(i)):
    quit: # Funktioniert in GUI nicht richtig...
    robot_name := "": # ...Daher auch Löschung des Roboternamens.
  end if:
end do:

# Dynamics Parameters
# Mass of each link
M_generic := Matrix(NL, 1):
for i from 1 to NL do
  M_generic[i,1]:=parse(sprintf("M%d", i-1)):
end do:
M := copy(M_generic):
if assigned(user_M) then
  if not RowDimension(user_M) = NL then
    printf("Input user_M is not of size %dx1. Error.\n", NL):
  end if:
  # Mass parameter given by user. Some entries can be zero or equal
  for i from 1 to NL do
    if user_M[i,1]<>M_generic[i,1] and user_M[i,1]<>0 then # check input
      printf("User input for mass %d should be either \"%s\" or zero. Not \"%s\". Continue anyway. \n", i-1, M_generic[i,1], user_M[i,1]):
    end if:
    M[i,1]:=user_M[i,1]:
  end do:
  printf("Masseparameter von Benutzer gesetzt. Eingabe für M:\n"):
  print(M);
end if:
# Center of Mass of each link (in link frame)
r_i_i_Si := Matrix(3, NL):
r_i_i_Si_generic := Matrix(3, NL):
for i from 1 to NL do
  r_i_i_Si_generic[1,i]:=parse(sprintf("SX%d", i-1)):
  r_i_i_Si_generic[2,i]:=parse(sprintf("SY%d", i-1)):
  r_i_i_Si_generic[3,i]:=parse(sprintf("SZ%d", i-1)):
end do:
if not assigned(user_CoM) then
  r_i_i_Si := r_i_i_Si_generic:
else
  if RowDimension(user_CoM) <> 3 or ColumnDimension(user_CoM) <> NL then
    printf("Input user_CoM is not of size 3x%d. Error.\n", NL):
  end if:
  for i from 1 to NL do
    for j from 1 to 3 do
      if user_CoM[j,i]<>r_i_i_Si_generic[j,i] and user_CoM[j,i]<>0 then # check input
        printf("User input for CoM %d,%d should be either \"%s\" or zero. Not \"%s\". Continue anyway. \n", j, i-1, r_i_i_Si_generic[j,i], convert(user_CoM[j,i],string)):
      end if:
    end do:
    r_i_i_Si[1..3,i]:=user_CoM[1..3,i]:
  end do:
  printf("Schwerpunktsparameter von Benutzer gesetzt. Eingabe für r_i_i_Si:\n"):
  print(r_i_i_Si);
end if:

# First Moment (mass and center of mass)
mr_i_i_Si := Matrix(3, NL):
mr_i_i_Si_generic := Matrix(3, NL):
xyzstrings := ["X", "Y", "Z"]:
for i from 1 to NL do
  for j from 1 to 3 do # loop over x-, y-, z-coordinates of the CoM
    mr_i_i_Si_generic[j,i]:= parse(sprintf("M%s%d", xyzstrings[j], i-1)):
  end do:
  if M[i,1] = 0 then next; end if: # Zero Mass can be set by the user. Then the first moment stays zero
  for j from 1 to 3 do # loop over x-, y-, z-coordinates of the CoM
    if r_i_i_Si[j,i] = parse(sprintf("S%s%d", xyzstrings[j], i-1)) then
      # default value for CoM is set (not overwritten by user input).
      # Set default value for first moment
      mr_i_i_Si[j,i]:= parse(sprintf("M%s%d", xyzstrings[j], i-1)):
    else
      # CoM is written by the user. Put this assumption also in the first moment to reduce dynamics parameters
      mr_i_i_Si[j,i] := M[i,1]*r_i_i_Si[j,i]:
    end if:
  end do:
end do:
if assigned(user_CoM) then
  printf("Schwerpunktsparameter von Benutzer gesetzt. Werte für mr_i_i_Si:\n"):
  print(mr_i_i_Si);
end if:

# Inertia of each link (about the center of mass, in link frame)
I_i_Si := Matrix(6, NL):
I_i_Si_generic := Matrix(6, NL):
for i from 1 to NL do
  if M[i,1] = 0 then next; end if: # Zero Mass can be set by the user. Then the inertia stays zero
  I_i_Si_generic[1,i]:=parse(sprintf("XXC%d", i-1)):
  I_i_Si_generic[2,i]:=parse(sprintf("XYC%d", i-1)):
  I_i_Si_generic[3,i]:=parse(sprintf("XZC%d", i-1)):
  I_i_Si_generic[4,i]:=parse(sprintf("YYC%d", i-1)):
  I_i_Si_generic[5,i]:=parse(sprintf("YZC%d", i-1)):
  I_i_Si_generic[6,i]:=parse(sprintf("ZZC%d", i-1)):
end do:
if not assigned(user_inertia) then
  I_i_Si := I_i_Si_generic:
else
  if RowDimension(user_inertia) <> 6 or ColumnDimension(user_inertia) <> NL then
    printf("Input user_inertia is not of size 6x%d. Error.\n", NL):
  end if:
  for i from 1 to NL do
    for j from 1 to 6 do
      value_ij_valid := false:
      for k from 1 to 6 do
        if user_inertia[j,i]=I_i_Si_generic[k,i] or user_inertia[j,i]=0 then # check input
          value_ij_valid := true:
          break:
        end if:
      end do:
      if not value_ij_valid then
        printf("User input for inertia %d,%d should be either \"%s\" or zero. Not \"%s\". Continue anyway. \n", j, i-1, convert(I_i_Si_generic[j,i],string), convert(user_inertia[j,i],string)):
      end if:
    end do:
    I_i_Si[1..6,i]:=user_inertia[1..6,i]:
  end do:
  printf("Trägheitsparameter von Benutzer gesetzt. Eingabe für I_i_Si:\n"):
  print(I_i_Si);
end if:

# Inertia of each link (about the origin of body frame, in link frame)
# Berechne die Inertial-Trägheitsmomente mit dem Steinerschen Verschiebungssatz
I_i_i_calc := Matrix(6, NL):
for i from 1 to NL do
  # Trägheitstensor (3x3) um den Körperschwerpunkt in Körper-KS
  I_i_Si_Tensor := Matrix([[I_i_Si[1, i], I_i_Si[2, i], I_i_Si[3, i]], [I_i_Si[2, i], I_i_Si[4, i], I_i_Si[5, i]], [I_i_Si[3, i], I_i_Si[5, i], I_i_Si[6, i]]]):
  # Steinerschen Satz anwenden
  I_i_i_Tensor := I_i_Si_Tensor + M[i,1]*Transpose(vec2skew(r_i_i_Si[1..3,i])) . vec2skew(r_i_i_Si[1..3,i]):
  # Wieder als Matrix abspeichern
  I_i_i_calc[..,i] := <I_i_i_Tensor[1,1]; I_i_i_Tensor[1,2]; I_i_i_Tensor[1,3]; I_i_i_Tensor[2,2]; I_i_i_Tensor[2,3]; I_i_i_Tensor[3,3]>:
end do:
# Allgemeine Form des Trägheitstensors (Einträge sind unabhängige Parameter)
I_i_i_generic := Matrix(6, NL):
for i from 1 to NL do
  I_i_i_generic[1,i]:=parse(sprintf("XX%d", i-1)):
  I_i_i_generic[2,i]:=parse(sprintf("XY%d", i-1)):
  I_i_i_generic[3,i]:=parse(sprintf("XZ%d", i-1)):
  I_i_i_generic[4,i]:=parse(sprintf("YY%d", i-1)):
  I_i_i_generic[5,i]:=parse(sprintf("YZ%d", i-1)):
  I_i_i_generic[6,i]:=parse(sprintf("ZZ%d", i-1)):
end do:
# Prüfe, welche Einträge des Trägheitstensors noch Schwerpunkts-Parameter enthalten. Diese müssen wieder auf die Standard-Werte mit unabhängigen Parametern gesetzt werden
I_i_i := Matrix(6, NL):
compstrings := ["XX", "XY", "XZ", "YY", "YZ", "ZZ"]:
for i from 1 to NL do
  if M[i,1] = 0 then next; end if: # Zero Mass can be set by the user. Then the inertia stays zero
  for j from 1 to 6 do # Alle Komponenten des Tensors
    # Nicht Null. Setze erstmal allgemeinen Eintrag:
    I_i_i[j,i] := I_i_i_generic[j,i]:
    # Prüfe, ob Schwerpunktsparameter vorkommt (durch Verschiebungssatz s.o.)
    IhasCoM := false:
    for k from 1 to 3 do
      if has(I_i_i_calc[j,i], r_i_i_Si_generic[k,i]) then
        IhasCoM := true: # Der Eintrag enthält die Schwerpunktskoordinaten. Nicht zulässig für weitere Rechnung.
        break:
      end if:
    end do:
    if has(I_i_i_calc[j,i], I_i_Si_generic[j,i]) then
        IhasCoM := true: # Es steht der Schwerpunktsbezogene Trägheitsterm drin. Weitere Rechnung damit nicht möglich.
    end if:
    # Prüfe, ob der Eintrag identisch zu einem vorherigen Eintrag ist. Das kommt vor, wenn aus Symmetriegründen Trägheitstensoren gleich sind
    Iisidentical := false:
    for k from 1 to j-1 do
      if I_i_i_calc[j,i] = I_i_i_calc[k,i] then
        I_i_i[j,i] := I_i_i[k,i]:
        Iisidentical := true:
        break:
      end if:
    end do:
    if IhasCoM = true or Iisidentical then
      next: # Es kommt ein Schwerpunktsparameter vor. Belasse Parameter auf allgemeinem Wert.
    end if:
    I_i_i[j,i] := I_i_i_calc[j,i]:
  end do:
end do:
if assigned(user_inertia) then
  printf("Trägheitsparameter von Benutzer gesetzt. Werte für I_i_i_calc:\n"):
  print(I_i_i_calc);
  printf("Trägheitsparameter von Benutzer gesetzt. Werte für I_i_i:\n"):
  print(I_i_i);
end if:
# Vector of stacked dynamics parameters for regressor form
# Matrix of link inertial parameters, stacked link parameter vectors.
# Diese Parameter-Matrix wird nur benutzt, um für die Generierung der Regressorform danach abzuleiten.
# Hier stehen also die allgemeinen ("generic") Parameter drin. Es ist egal, ob diese bereits durch den Benutzer zu Null gesetzt sind.
# Dann würden die allgemeinen Parameter nicht in der Dynamik vorkommen und die Ableitung wäre Null.
PV2_mat := Matrix(NL, 10):
for i to NL do 
  PV2_mat[i, 1 .. 6] := I_i_i_generic[1 .. 6, i]:
  PV2_mat[i, 7 .. 9] := mr_i_i_Si_generic[1 .. 3, i]:
  PV2_mat[i, 10] := M_generic[i, 1]:
end do:

# Parameter-Vektor Erstellen: vector of link inertial parameters (delta in [1]).
# Gleiche Überlegung wie für Parameter-Matrix
PV2_vec := Matrix(10*(NL), 1):
for i to NL do 
  for j to 10 do 
    PV2_vec[10*(i-1)+j] := PV2_mat[i, j]:
  end do:
end do:
# Kinematische Zwangsbedingungen
# Prüfe, ob kinematische Zwangsbedingungen in der Roboterkonfiguration genannt sind durch Prüfung der Existenz der entsprechenden Variablen.
if type( kintmp_t, 'Matrix') = false then
  kintmp_t := Matrix(1,1): # Dummy-Werte damit später alles funktioniert
  kintmp_s := Matrix(1,1):
end if:
# Export
save q_t, q_s, qD_t, qD_s, qDD_t, qDD_s, qJ_t, qJ_s, qJD_t, qJD_s, qJDD_t, qJDD_s, g_world, X_base_t, X_base_s, V_base_t, V_base_s, VD_base_t, VD_base_s, qoffset, theta, alpha, d, a,v,b,beta, sigma,mu,M, r_i_i_Si, mr_i_i_Si, I_i_i, I_i_Si, PV2_vec, PV2_mat, robot_name, NQ,NQB,NQJ,NJ,NL, base_method_name, T_basevel, kintmp_t, kintmp_s, sprintf("../codeexport/%s/tmp/tree_floatb_twist_definitions", robot_name):
save q_t, q_s, qD_t, qD_s, qDD_t, qDD_s, qJ_t, qJ_s, qJD_t, qJD_s, qJDD_t, qJDD_s, g_world, X_base_t, X_base_s, V_base_t, V_base_s, VD_base_t, VD_base_s, qoffset, theta, alpha, d, a,v,b,beta, sigma,mu,M, r_i_i_Si, mr_i_i_Si, I_i_i, I_i_Si, PV2_vec, PV2_mat, robot_name, NQ,NQB,NQJ,NJ,NL, base_method_name, T_basevel, kintmp_t, kintmp_s, sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name):
# Einzelne DH-Parameter als Matlab-Code exportieren. Damit lässt sich in Matlab ein passender Parametersatz generieren.
# Die folgenden Dateien müssen immer generiert werden, wenn die Bash-Skripte zur Funktionszusammensetzung fehlerfrei laufen sollen.
# Benutze die Funktion convert_t_s, um eventuelle Substitutionsvariablen für konstante Gelenkwinkel zu verwenden, da die Matlab-Terme auch mit substituierten Ausdrücken generiert werden.
# Zur Kennzeichnung von zeitabhängigen und konstanten Ausdrücken kann "delta1(t)", "delta1" und "delta1s" verwendet werden.
MatlabExport(v, sprintf("../codeexport/%s/tmp/parameters_mdh_v_matlab.m", robot_name), 2):
MatlabExport(convert_t_s(a), sprintf("../codeexport/%s/tmp/parameters_mdh_a_matlab.m", robot_name), 2):
d_export := d *~ (1-~sigma):
MatlabExport(convert_t_s(d_export), sprintf("../codeexport/%s/tmp/parameters_mdh_d_matlab.m", robot_name), 2):
theta_export := theta *~ sigma:
MatlabExport(convert_t_s(theta_export), sprintf("../codeexport/%s/tmp/parameters_mdh_theta_matlab.m", robot_name), 2):
MatlabExport(convert_t_s(b), sprintf("../codeexport/%s/tmp/parameters_mdh_b_matlab.m", robot_name), 2):
MatlabExport(convert_t_s(alpha), sprintf("../codeexport/%s/tmp/parameters_mdh_alpha_matlab.m", robot_name), 2):
MatlabExport(convert_t_s(beta), sprintf("../codeexport/%s/tmp/parameters_mdh_beta_matlab.m", robot_name), 2):
MatlabExport(convert_t_s(qoffset), sprintf("../codeexport/%s/tmp/parameters_mdh_qoffset_matlab.m", robot_name), 2):
MatlabExport(sigma, sprintf("../codeexport/%s/tmp/parameters_mdh_sigma_matlab.m", robot_name), 2):
MatlabExport(mu, sprintf("../codeexport/%s/tmp/parameters_mdh_mu_matlab.m", robot_name), 2):

# Einzelne Dynamikparameter als Matlab-Code exportieren. Wenn die Parameter durch Benutzereingaben verändert wurden, lässt sich diese Information so weiter benutzen.
# (z.B. in der Definition von Eingabeparametern in den Testskripten).
# In den Maple-Variablen ist die Spalte der Index der Körper und die Zeilen sind die Indizes für die xyz-Komponenten (einfacherer Aufruf der Variablen)
# In Matlab ist es umgekehrt (führt zu konsistenterem Code in Matlab): Die Zeilen entsprechen dem Körper-Index. Daher hier Transponierung.
# Zusätzlich ist die Reihenfolge der Komponenten der Trägheitstensoren in Matlab und Maple unterschiedlich (daher die Indizierung).
# Matlab: xx, yy, zz, xy, xz, yz (erst Hauptmomente, dann Deviationsmomente)
# Maple: xx, xy, xz, yy, yz, zz (Dreiecksform)
MatlabExport(M, sprintf("../codeexport/%s/tmp/parameters_dyn_mges_matlab.m", robot_name), 2):
MatlabExport(Transpose(r_i_i_Si), sprintf("../codeexport/%s/tmp/parameters_dyn_rSges_matlab.m", robot_name), 2):
MatlabExport(Transpose(I_i_Si([1,4,6,2,3,5],..)), sprintf("../codeexport/%s/tmp/parameters_dyn_Icges_matlab.m", robot_name), 2):
MatlabExport(Transpose(mr_i_i_Si), sprintf("../codeexport/%s/tmp/parameters_dyn_mrSges_matlab.m", robot_name), 2):
MatlabExport(Transpose(I_i_i([1,4,6,2,3,5],..)), sprintf("../codeexport/%s/tmp/parameters_dyn_Ifges_matlab.m", robot_name), 2):

