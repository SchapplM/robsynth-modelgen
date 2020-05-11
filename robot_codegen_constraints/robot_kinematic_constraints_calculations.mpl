
# Berechnungen für Systeme mit kinematischen Zwangsbedingungen
# Beschreibung
# Dieses Arbeitsblatt ist für Systeme mit kinematischen Zwangsbedingungen relevant.
# Es werden die Zusammenhänge zwischen den Minimalkoordinaten des Systems mit Zwangsbedingungen mit dem offenen System ohne Zwangsbedingungen hergestellt (nach [NakamuraGho1989], [ParkChoPlo1999]).
# Das System ohne Zwangsbedingungen kann durch die Erstellung einer neuen robot_env definiert werden (Weglassen der Zwangsbedingungen, offene Baumstruktur).
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# kinematic_constraints_calculations -> Berechnungen bezüglich kinematischer Zwangsbedingungen
# Quellen
# [KhalilKle1986] Khalil, W. & Kleinfinger, J.: A new geometric notation for open and closed-loop robots (1986)
# [NakamuraGho1989] Nakamura, Yoshihiko and Ghodoussi, Modjtaba: Dynamics computation of closed-link robot mechanisms with nonredundant and redundant actuators (1989)
# [ParkChoPlo1999] Park, FC and Choi, Jihyeon and Ploen, SR: Symbolic formulation of closed chain dynamics in independent coordinates (1999)
# Autor
# Moritz Schappler, schappler@irt.uni-hannover.de, 2017-12
# Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# Initialisierung
interface(warnlevel=0): # Unterdrücke die folgende Warnung.
restart: # Gibt eine Warnung, wenn über Terminal-Maple mit read gestartet wird.
interface(warnlevel=3):
interface(rtablesize=30):
with(StringTools): # Für Zeitausgabe
with(LinearAlgebra):
with(codegen):
with(CodeGeneration):
codegen_act := true:
codegen_opt := 2: # Hoher Optimierungsgrad.
;
read "../helper/proc_MatlabExport":
read "../helper/proc_simplify2":
read "../helper/proc_convert_s_t":
read "../helper/proc_convert_t_s":
with(RealDomain): # Schränkt alle Funktionen auf den reellen Bereich ein. Muss nach Definition von MatlabExport kommen. Sonst geht dieses nicht.
;
read "../robot_codegen_constraints/proc_subs_kintmp_exp":
# Definition und Zwangsbedingeun
read "../robot_codegen_definitions/robot_env":
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name):
if not assigned(simplify_options) or simplify_options(1)=-1 then
  if NJ <= 5 then
    use_simplify := 1: # Nur bei sehr einfachen Systemen (Viergelenkkette). Sonst hängt es sich auf.
  else
    use_simplify := 0: # Sonst keine Vereinfachung.
  end if:
else
  use_simplify := simplify_options(1): # erster Eintrag ist für Zwangsbedingungen
end if:
# Lesen der Zwangsbedingungen
kin_constraints_exist := false:
constrfile := sprintf("../codeexport/%s/tmp/kinematic_constraints_maple_inert.m", robot_name):
if FileTools[Exists](constrfile) then
  read constrfile:
end if:
if kin_constraints_exist = true then:
  kintmp_qs := kintmp_qs: # gelesene Variable sonst nicht sichtbar
  kintmp_qt := kintmp_qt: # gelesene Variable sonst nicht sichtbar
  kintmp_subsexp := kintmp_subsexp: # gelesene Variable sonst nicht sichtbar
  printf("%s. Kinematische Zwangsbedingungen gelesen.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
else
  printf("%s. Es gibt keine Zwangsbedingungen. Offene Struktur. Keine weiteren Berechnungen notwendig.", FormatTime("%Y-%m-%d %H:%M:%S")):
  restart: # Funktioniert nicht. Gedacht, damit im worksheet-Modus nichts mehr passieren kann.
  robot_name := "": # Damit werden die Export-Befehle ungültig
  codegen_act := false:
  quit:
end if:
# Abhängige Koordinaten definieren
# Nicht alle Ersetzungsausdrücke sind auch Koordinaten
NSE := RowDimension(kintmp_s):
# temporäre kinematische Ausdrücke (die auch die Zwangsbedingungen enthalten)
kintmp_s:
# Gelenkvariable für Drehgelenke (nach [KhalilKle1986])
theta_s := convert_t_s(theta):
# Gelenkvariable für Schubgelenke (nach [KhalilKle1986])
d_s := convert_t_s(d):
# Finde und zähle die Vorkommnisse von kintmp_s in theta_s und d_s. Die Anzahl der abhängigen Gelenkvariablen muss mit den definierten Minimalkoordinaten übereinstimmen, sonst ist das benutzerspezifizierte Modell falsch.
# Zähle, welche Zwangsbedingungen tatsächlich vorkommen
kintmp_zaehler := Matrix(NSE,1):
for i from 1 to NSE do
  if has(has~(<theta_s(1..NJ,1);d_s(1..NJ,1)>, kintmp_s(i,1)), true) then
    kintmp_zaehler(i,1) := 1:
  end if:
end do:
# Zähle, welche Gelenke abhängig sind (durch ZB beschrieben).
# (in der Gelenkvariablen stehen die Inhalte von kintmp)
jv_zaehler := Matrix(NJ,1):
for i from 1 to NJ do
  if has(has~(theta_s(i,1), kintmp_s), true) or has(has~(d_s(i,1), kintmp_s), true) then
    jv_zaehler(i,1) := 1:
  end if:
end do:
# Zählen, welche Gelenke statische Transformationen sind
sigma2_zaehler := Matrix(NJ,1):
for i from 1 to NJ do
  if sigma(i,1)=2 then
    sigma2_zaehler(i,1) := 1:
  end if:
end do:
# Anzahl der abhängigen Gelenkvariablen (Zählmethode: Gelenkkoordinaten durchgehen)
NC1 := add(jv_zaehler(n), n = 1 .. NJ):
# Anzahl der abhängigen Gelenkvariablen (Zählmethode: Zwangsbedingungen durchgehen)
NC2 := add(kintmp_zaehler(n), n = 1 .. NSE):
# Anzahl der konstanten Transformationen
NS := add(sigma2_zaehler(n), n = 1 .. NJ):
# Anzahl der "realen" Gelenke (statische Transformationen herausrechnen)
NRJ := NJ - NS:
# Es gibt (NL - 1) durch Gelenke bewegte Körper, NVJ/2 Koordinatensysteme (hier als "Gelenk" gezählt) sind statisch aufgrund der Schleifenschlussbedingungen
if NQJ+NC1 <> NRJ then
  printf("%s. Fehler: Anzahl der Abhängigen Gelenke %d (aus Gelenkvariablen) passt nicht zur Anzahl der Gelenke %d und der Minimalkoordinaten %d\n", FormatTime("%Y-%m-%d %H:%M:%S"), NC1, NRJ, NQJ):
end if:
if NC1 <> NC2 then
  printf("%s. Fehler: Anzahl der Abhängigen Gelenke %d (aus Gelenkvariablen) passt nicht zur Anzahl der Zwangsbedingungen %d (aus Verwendung der Zwangsbedingungs-Variablen)\n", FormatTime("%Y-%m-%d %H:%M:%S"), NC1, NC2):
end if:
# Indizes der Zwangsbedingungen (in kintmp_s), die wirklich verwendet werden. Wird später nicht mehr benötigt.
Ind_ZB := Matrix(NC,1):
j := 1:
for i from 1 to NSE do
  if kintmp_zaehler(i,1) = 1 then
    Ind_ZB(j,1) := i:
    j := j + 1:
  end if:
end do:
# Koordinaten der offenen Struktur (MDH-Gelenkvariablen)
# Darstellung der Gelenkvariablen in abhängigkeit der Minimalkoordinaten. Diese Gelenkvariablen sind die Minimalkoordinaten des offenen Systems (ohne Zwangsbedingungen)
jv_s := theta_s*~(Matrix(NJ,1,1)-sigma) + d_s*~sigma - qoffset:
jv_qs := jv_s: # Initialisierung
jv_qt := Matrix(NJ,1): # Initialisierung
;
# Ersetzen der abhängigen Gelenkkoordinaten
for i from 1 to NJ do # Index über alle Gelenkvariablen
  # Substituiere sin und cos der Winkel (einfachere Ausdrücke. Sollten aber hier nicht drin sein.)
  jv_qs(i,1) := subs_kintmp_exp(jv_qs(i,1)):
  # Substituiere die verbleibenden Winkel direkt
  for jj from 1 to NSE do # Index über zu ersetzende Winkel
    jv_qs(i,1) := subs( { kintmp_s(jj, 1) = kintmp_qs(jj, 1) }, jv_qs(i,1) ): 
  end do:
  jv_qt(i,1) := convert_s_t( jv_qs(i,1) ):
end do:
# Jacobi-Matrix der abhängigen Winkel
# Ableitung der Gelenkkoordinaten nach den Minimalkoordinaten.
# Wird benötigt, um die Geschwindigkeit der Gelenkkoordinaten der offenen Struktur zu berechnen und um die Momente der offenen Struktur auf die geschlossene Struktur umzurechnen.
# Ist Matrix Phi aus [ParkChoPlo1999]. qJ hier ist qa dort. qr dort entspricht den Koordinaten desselben Systems ohne Zwangsbedingungen.
# Entspricht der Matrix W aus [NakamuraGho1989] (nur dass hier die Aufteilung q1,q2 nicht gestapelt in q auftreten muss).
# Wenn statische Transformationen definiert sind (mit sigma=2), dann werden die entsprechenden Spalten hier trotzdem angegeben (das ist nicht unbedingt intuitiv, aber von der Programmierung bedeutend einfach)
Phi_s := Matrix(NJ, NQJ):
# Jacobi-Matrix berechnen
for i from 1 to NJ do
  for j from 1 to NQJ do
    Phi_s(i,j) := diff(jv_qs(i,1), qJ_s(j,1)):
  end do:
end do:
# Term vereinfachen
if use_simplify >= 1 then
  tmp_t1 := time(): tmp_l1 := length(Phi_s):
  printf("%s. Vereinfache Passiv-Gelenk-Jacobi (Dimension %d x %d). Länge: %d.\n", \
    FormatTime("%Y-%m-%d %H:%M:%S"), NJ, NQJ, tmp_l1):
  for i from 1 to NJ do
    for j from 1 to NQJ do
      tmp_t1ij := time(): tmp_l1ij := length(Phi_s(i,j)):
      printf("%s. Vereinfache Passiv-Gelenk-Jacobi Eintrag %d,%d. Länge: %d.\n", \
        FormatTime("%Y-%m-%d %H:%M:%S"), i, j, tmp_l1ij):
      Phi_s(i,j) := simplify2(Phi_s(i,j)):
      tmp_t2ij := time(): tmp_l2ij := length(Phi_s(i,j)):
      printf("%s. Passiv-Gelenk-Jacobi der ZB Eintrag %d,%d vereinfacht. Länge: %d->%d. Rechenzeit %1.1fs.\n", \
        FormatTime("%Y-%m-%d %H:%M:%S"), i, j, tmp_l1ij, tmp_l2ij, tmp_t2ij-tmp_t1ij):
    end do:
  end do:
  tmp_t2 := time(): tmp_l2 := length(Phi_s):
  printf("%s. Passiv-Gelenk-Jacobi der ZB vereinfacht. Länge: %d->%d. Rechenzeit %1.1fs.\n", \
    FormatTime("%Y-%m-%d %H:%M:%S"), tmp_l1, tmp_l2, tmp_t2-tmp_t1):
end if:
# Zeitableitung der Jacobi-Matrix
# Wird benötigt, um die Beschleunigung der Gelenkkoordinaten der offenen Struktur zu berechnen.
Phi_t := convert_s_t(Phi_s):
PhiD_t := diff~(Phi_t, t):
PhiD_s := convert_t_s(PhiD_t):
# Exportiere Code für folgende Skripte
# Speichere Maple-Ausdruck (Eingabe-Format und internes Format)
save jv_s, jv_qs, jv_qt, sprintf("../codeexport/%s/tmp/kinematic_constraints_explicit_maple.m", robot_name):
save Phi_s, sprintf("../codeexport/%s/tmp/kinematic_constraints_explicit_jacobian_maple.m", robot_name):
save PhiD_s, sprintf("../codeexport/%s/tmp/kinematic_constraints_explicit_jacobian_time_derivative_maple.m", robot_name):
printf("%s. Ausdrücke für Kinematische ZB gespeichert (Maple)\n", FormatTime("%Y-%m-%d %H:%M:%S")):
if codegen_act = true then
  MatlabExport(jv_zaehler, sprintf("../codeexport/%s/tmp/kinconstr_index_dependant_joints_matlab.m", robot_name), 2):
  MatlabExport(jv_qs, sprintf("../codeexport/%s/tmp/kinconstr_expl_matlab.m", robot_name), 2):
  MatlabExport(Phi_s, sprintf("../codeexport/%s/tmp/kinconstr_expl_jacobian_matlab.m", robot_name), 2):
  MatlabExport(PhiD_s, sprintf("../codeexport/%s/tmp/kinconstr_expl_jacobianD_matlab.m", robot_name), 2):
  printf("%s. Ausdrücke für Kinematische ZB gespeichert (Matlab)\n", FormatTime("%Y-%m-%d %H:%M:%S")):
end if:



