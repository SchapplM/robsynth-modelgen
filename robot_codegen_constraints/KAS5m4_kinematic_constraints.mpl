# Kinematik-Berechnung 3. Arm KAS5
# Beschreibung
# Berechnung der Kinematik der parallelen Struktur: Stellung des Kompensationsmechanismus in Abhängigkeit der gemessenen Winkel der Haupt-Armkinematik
# Zu Geometrie und Winkeln, siehe Bild (Unterlagen)
# 
# Modell m4: Kurbel N7 ist drehbar gegen Körper K2 gelagert (über die Achse)
# 
# Berechne alle notwendigen Winkel in kintmp in Abhängigkeit konstanter Größen und der verallgemeinerten Koordinaten
# 
# Berechne zuerst nur Sinus und Cosinus jedes Winkels. Diese werden für die Rotationsmatrizen benötigt.
# Die Berechnung der Winkel selbst (zur Kontrolle) benötigt die arctan-Funktion, die längere Ausdrücke nur sehr langsam auswertet.
# Daher werden zunächst alle Winkel mit %arctan (inert-Funktion berechnet) und am Ende erst ausgewertet.
# Die Zeitableitung der Winkel (in späteren Skripten) wird aus der inert-Funktion berechnet.
# 
# Hinweise:
# Der Befehl arctan hat die Funktionalität von atan2
# 
# Eingabe:
# "../codeexport/KAS5_m4_definitions"
# * enthält die Definitionen aller Konstanten und verallgemeinerter Koordinaten
# 
# Ausgabe:
# "../codeexport/%s_kinematic_constraints_maple_inert.m"
# * Erzeugt die Ausdrücke für Winkel der Parallelstruktur (kintmp), Ersetzungsausdrücke für sin,cos (kintmp_subsexp) und die Federlänge (lpar) in Abhängigkeit der verallgemeinerten Koordinaten.
# Quellen
# [1] Wang, Jiegao and Gosselin, ClémentM.: A New Approach for the Dynamic Analysis of Parallel Manipulators (1998)
# Autor
# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-04
# Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# Initialisierung
restart:
kin_constraints_exist := true: # Für Speicherung
with(StringTools): # Für Zeitausgabe
with(LinearAlgebra):
with(codegen):
with(CodeGeneration):
codegen_act := true:
codegen_opt := 1: # Geringerer Optimierungsgrad. Sonst zu lange.
codegen_debug := 0: # Zur Code-Generierung auch für Nicht-Inert-Ausdrücke
read "../helper/proc_MatlabExport":
read "../transformation/proc_rotx":
read "../transformation/proc_roty":
read "../transformation/proc_rotz":
read "../helper/proc_convert_s_t":
read "../helper/proc_convert_t_s":
read "../robot_codegen_constraints/proc_subs_kintmp_exp":
read "../helper/proc_intersect_circle":
with(RealDomain): # Schränkt alle Funktionen auf den reellen Bereich ein. Muss nach Definition von MatlabExport kommen. Sonst geht dieses nicht.
read "../robot_codegen_definitions/robot_env":
read sprintf("../codeexport/%s_tree_floatb_definitions", robot_name):
# Variable mit Winkeln der Nebenstruktur nur in Abhängigkeit der verallgemeinerten Koordinaten
kintmp_qs := Matrix(RowDimension(kintmp_s),1):
# Konstante Winkel bereits hineinschreiben
# TODO: Hier womöglich Fehler: Substituierte Ausdrücke werden nach der Zeit abgeleitet.
for i from 1 to RowDimension(kintmp_s) do
  if diff(kintmp_s(i,1), t) = 0 then
    kintmp_qs(i,1) := kintmp_s(i,1):
  end if:
end do:
# Variablen definieren für die Hilfswinkel
gamma5_s := kintmp_s(5,1):
delta_s := kintmp_s(6..23,1):
delta_qs := kintmp_qs(6..23,1):
# Ersetzungsausdrücke definieren.
# Speichere Sinus und Cosinus der Winkel direkt ab, da diese in den Rotationsmatrizen direkt auftreten.
# Spalte 1: Zu suchender Ausdruck (sin oder cos eines Winkels)
# Spalte 2: Einzusetzender Ausdruck.
# Dadurch werden arctan-Ausdrücke in der direkten Kinematik reduziert.
# Ähnliches Vorgehen wie in [1].
kintmp_subsexp := Matrix(2*RowDimension(kintmp_s),2):
for i from 1 to RowDimension(kintmp_s) do
  kintmp_subsexp(2*i-1, 1) := sin(kintmp_s(i,1)):
  kintmp_subsexp(2*i,   1) := cos(kintmp_s(i,1)):
  # Initialisierung der rechten Spalte mit gleichen Werten. Später nur Ersetzung, wenn Vorteilhaft.
  kintmp_subsexp(2*i-1, 2) := kintmp_subsexp(2*i-1, 1):
  kintmp_subsexp(2*i,   2) := kintmp_subsexp(2*i,   1):
end do:
# Annahmen über Konstanten: Längen sind alle Positiv.
# (Wird anscheinend für arctan gebraucht). Ansonsten steht csgn in exportiertem Code.
assume(l1>0):
assume(l2>0):
assume(l3>0):
assume(l4>0):
assume(l5>0):
assume(l6>0):
assume(l7>0):
assume(l8>0):
assume(l9>0):
assume(l10>0):
assume(l11>0):
assume(l12>0):
assume(l13>0):
assume(l14>0):
assume(l15>0):
assume(l16>0):
assume(l17>0):
assume(l18>0):
assume(l19>0):
assume(l20>0):
assume(l21>0):
assume(l22>0):
# Schalter mit Optionen für die Code-Generierung.
# Wähle, ob trigonometrische arctan-Ausdrücke ausgewertet werden, oder ob Sinus- und Cosinussatz angewendet werden sollen.
# Dies hat großen Einfluss auf die Rechendauer.
# Die Optimierungen sind so gewählt, dass
# * Kreisschnittpunktsfunktionen nicht mit den komplizierten Eingabedaten aufgerufen, sondern mit allgemeinen Platzhaltern, die dann ersetzt werden,
# * Keine Zwischenoptimierungen durchgeführt werden. Durch simplify und combine werden die symbolischen Ausdrücke größer, was später die Rechenzeit enorm erhöht
# * Sinus und Cosinus der Parallelwinkel ersetzt werden und kein sin(arctan(...)) ausgerechnet wird.
Schalter_Opt := <1;1;0;0;0;0;0;1>:
printf("Beginn der Berechnungen. %s\n", FormatTime("%Y-%m-%d %H:%M:%S")):
st := time():
# Reihenfolge, in der die Winkel der Parallelstruktur berechnet werden. Hilft beim exportieren von Code, wenn beim Debuggen in der gleichen Reihenfolge ausgegeben wird.
Reihenfolge_kintmp := <12; 5; 21; 4; 9; 16; 3; 8; 24; 11; 2; 1; 6; 7; 10; 13; 14; 15; 17; 18; 19; 20; 22; 23>:
# Berechne delta7
# Gl. (30)
delta_qs(7) := kintmp_qs(22) + convert_t_s(theta(4)):
kintmp_qs(12) := delta_qs(7):
# Berechne gamma5
# Gl.(2)
gamma5_qs := - kintmp_qs(12) + Pi + convert_t_s(theta(5)):
kintmp_qs(5) := gamma5_qs:
kintmp_subsexp(9,2) := sin(gamma5_qs):
kintmp_subsexp(10,2) := cos(gamma5_qs):
# Berechne delta16
# Kreisschnittpunkt berechnen
r_4_G4_G5 := <l11;0;0>:
r_12_G5_B := <l14;0;0>:
R_4_12 := rotz(Pi+delta_qs(7)):
r_4_G5_B := R_4_12 . r_12_G5_B:
save r_4_G5_B, "../codeexport/KAS5_m4_kinematic_constraints_r_4_G5_B_debug_maple":
R_3_4 := rotz(convert_t_s(theta(4))):
R_4_3 := Transpose(R_3_4):
# Vektor D-G4 (ausgedrückt in KS 3)
r_3_D_G4 := <l5-l6;0;0>:
r_4_G4_D := R_4_3 . (-r_3_D_G4):
# Vektor G4-G5 (ausgedrückt in KS 4)
r_4_G4_G5 := <l11;0;0>:
r_4_G5_D := -r_4_G4_G5 + r_4_G4_D:
save r_4_G5_D, "../codeexport/KAS5_m4_kinematic_constraints_r_4_G5_D_noopt_debug_maple":
if Schalter_Opt(7) = 1 then
  r_4_G5_D := simplify(r_4_G5_D, trig):
  r_4_G5_D := simplify(r_4_G5_D):
  r_4_G5_D := combine(r_4_G5_D, trig):
end if:
save r_4_G5_D, "../codeexport/KAS5_m4_kinematic_constraints_r_4_G5_D_opt_debug_maple":
if Schalter_Opt(8) = 1 then
  # Rufe die Kreisschnittpunkt-Funktion mit allgemeinen Werten auf und ersetze das Ergebnis.
  subs_ic := <r_4_G5_B_Allg_x; r_4_G5_B_Allg_y; r_4_G5_D_Allg_x; r_4_G5_D_Allg_y>:
  subs_ic_KAS := <r_4_G5_B(1..2,1); r_4_G5_D(1..2,1)>:
  ReturnValue_Allg := intersect_circle(<r_4_G5_B_Allg_x; r_4_G5_B_Allg_y>, <r_4_G5_D_Allg_x; r_4_G5_D_Allg_y>, l13, l22):
  ReturnValue := ReturnValue_Allg:
  for i from 1 to 4 do 
    ReturnValue := subs({subs_ic(i) = subs_ic_KAS(i)}, ReturnValue):
  end do:
else
  ReturnValue := intersect_circle(r_4_G5_B, r_4_G5_D, l13, l22):
end if:
printf("Erster Kreisschnittpunkt berechnet. %s. CPU-Zeit bis hier: %1.2fs.\n", FormatTime("%Y-%m-%d %H:%M:%S"), time()-st):
r_4_G5_C := <ReturnValue(1..2, 2); 0>: # Alternative 2
save r_4_G5_C, "../codeexport/KAS5_m4_kinematic_constraints_r_4_G5_C_debug_noopt_maple":
if Schalter_Opt(4) = 1 then
  r_4_G5_C := simplify(r_4_G5_C, trig):
  r_4_G5_C := simplify(r_4_G5_C):
  r_4_G5_C := combine(r_4_G5_C, trig):
end if:
printf("CPU-Zeit bis hier: %1.2fs\n", time()-st):
save r_4_G5_C, "../codeexport/KAS5_m4_kinematic_constraints_r_4_G5_C_debug_opt_maple":
r_4_D_C := - r_4_G5_D + r_4_G5_C:
r_3_D_C := R_3_4 . r_4_D_C:
save r_3_D_C, "../codeexport/KAS5_m4_kinematic_constraints_r_3_D_C_debug_noopt_maple":
if Schalter_Opt(6) = 1 then
  r_3_D_C := simplify(r_3_D_C, trig):
  r_3_D_C := simplify(r_3_D_C):
  r_3_D_C := combine(r_3_D_C, trig):
end if:
save r_3_D_C, "../codeexport/KAS5_m4_kinematic_constraints_r_3_D_C_debug_opt_maple":
# Gl. (m3.5)
delta_qs(16) := %arctan(-r_3_D_C(2), r_3_D_C(1)):
sin_delta16_qs := -r_3_D_C(2)/l22:
cos_delta16_qs := r_3_D_C(1)/l22:
kintmp_qs(21) := delta_qs(16):
kintmp_subsexp(41,2) := sin_delta16_qs:
kintmp_subsexp(42,2) := cos_delta16_qs:
# Berechne gamma3
# Berechnungen für Hebel E-F-G3
# Zweiter Kreismittelpunkt für Schnittpunkt-Algorithmus
r_3_G3_D := <l6;0;0>:
r_3_D_G3 := -r_3_G3_D:
r_10_D_E := rotz(delta_qs(9)) . <l21;0;0>:
# Stelle Rotationsmatrix mit Zeitabhängigen Winkel delta16 auf (ohne Ersetzung mit delta16(q), da dort arctan vorkommt.
# Ersetze sin(delta16), cos(delta16) mit Ausdrücken von oben. Bei arctan(sin(...)) entstehen kompliziertere Ausdrücke
if Schalter_Opt(1) = 1 then
  R_3_10_s := rotz(- delta_s(16)):
  R_3_10 := subs_kintmp_exp(R_3_10_s):
else
  R_3_10 := rotz(- delta_qs(16)):
end if:
r_3_D_E := R_3_10 . r_10_D_E:
save r_3_D_E, sprintf("../codeexport/%s_kinematic_constraints_r_3_D_E_noopt_debug_maple", robot_name):
if Schalter_Opt(5) = 1 then
  r_3_D_E := simplify(r_3_D_E, trig):
  r_3_D_E := simplify(r_3_D_E):
  r_3_D_E := combine(r_3_D_E, trig):
end if:
save r_3_D_E, sprintf("../codeexport/%s_kinematic_constraints_r_3_D_E_opt_debug_maple", robot_name):
if Schalter_Opt(2) = 1 then
  # Rufe die Kreisschnittpunkt-Funktion mit allgemeinen Werten auf und ersetze das Ergebnis.
  # dies führt zu einer wesentlichen Verringerungen der Rechenzeit, da das Einsetzen und Weiterrechnen der komplizierten Ausdrücke in der Unterfunktion entfällt.
  subs_ic := <r_3_D_E_Allg_x; r_3_D_E_Allg_y; r_3_D_G3_Allg_x; r_3_D_G3_Allg_y>:
  subs_ic_KAS := <r_3_D_E(1..2,1); r_3_D_G3(1..2,1)>:
  ReturnValue_Allg := intersect_circle(<r_3_D_E_Allg_x; r_3_D_E_Allg_y>, <r_3_D_G3_Allg_x; r_3_D_G3_Allg_y>, l20, l4):
  ReturnValue := ReturnValue_Allg:
  for i from 1 to 4 do 
    ReturnValue := subs({subs_ic(i) = subs_ic_KAS(i)}, ReturnValue):
  end do:
else
  ReturnValue := intersect_circle(r_3_D_E, r_3_D_G3, l20, l4):
end if:
printf("Zweiter Kreisschnittpunkt berechnet. %s. CPU-Zeit bis hier: %1.2fs.\n", FormatTime("%Y-%m-%d %H:%M:%S"), time()-st):
r_3_D_F := <ReturnValue(1..2, 2); 0>: # Alternative 2
save r_3_D_F, sprintf("../codeexport/%s_kinematic_constraints_r_3_D_F_noopt_debug_maple", robot_name):
if Schalter_Opt(3) = 1 then
  r_3_D_F := simplify(r_3_D_F, trig):
  r_3_D_F := simplify(r_3_D_F):
  r_3_D_F := combine(r_3_D_F, trig):
end if:
r_3_F_G3 := -r_3_D_F + r_3_D_G3:
printf("CPU-Zeit bis hier: %1.2fs\n", time()-st):
save r_3_D_F, sprintf("../codeexport/%s_kinematic_constraints_r_3_D_F_opt_debug_maple", robot_name):
save ReturnValue, r_3_D_F, r_3_F_G3, sprintf("../codeexport/%s_kinematic_constraints_gamma3_qs_debug_maple", robot_name):
# Gl. (m3.6)
gamma3_qs := %arctan(r_3_F_G3(2,1), -r_3_F_G3(1,1)):
sin_gamma3_qs := r_3_F_G3(2,1) / l4:
cos_gamma3_qs := (-r_3_F_G3(1,1)) / l4:
kintmp_qs(4) := gamma3_qs:
kintmp_subsexp(7,2) := sin_gamma3_qs:
kintmp_subsexp(8,2) := cos_gamma3_qs:
save ReturnValue, r_3_D_F, r_3_F_G3, gamma3_qs, sprintf("../codeexport/%s_kinematic_constraints_gamma3_qs_debug_maple", robot_name):
save kintmp_qs, kintmp_subsexp, sprintf("../codeexport/%s_kinematic_constraints_maple_debug_nach_gamma3.m", robot_name):
printf("Winkel gamma3 berechnet. %s. CPU-Zeit bis hier: %1.2fs.\n", FormatTime("%Y-%m-%d %H:%M:%S"), time()-st):
# Berechne delta4
# Gl. (22)
delta_qs(4) := Pi/2 - delta_qs(16) - delta_qs(12):
kintmp_qs(9) := delta_qs(4):
# Berechne delta11
# Gl. (3)
delta_qs(11) := delta_qs(10) - delta_qs(4):
kintmp_qs(16) := delta_qs(11):
# Berechne beta2
# Gl. (m3.7)
# Siehe auch: KAS5_m3_sym_codegen_kinematic_constraints_hilfsberechnungen.mw
d7b2 := %arctan((sin(delta_qs(7)) * l14 + r_4_G5_C(2)), (cos(delta_qs(7)) * l14 + r_4_G5_C(1))):
beta2_qs := d7b2 - delta_qs(7):
kintmp_qs(3) := beta2_qs:
# Nutze Additionstheoreme zur Bestimmung von Sinus und Cosinus.
cos_d7b2 := (cos(delta_qs(7)) * l14 + r_4_G5_C(1)) / l13:
sin_d7b2 := (sin(delta_qs(7)) * l14 + r_4_G5_C(2)) / l13:
sin_beta2_qs := sin_d7b2*cos(delta_qs(7)) - cos_d7b2*sin(delta_qs(7)):
cos_beta2_qs := cos_d7b2*cos(delta_qs(7)) + sin_d7b2*sin(delta_qs(7)):
kintmp_subsexp(5,2) := sin_beta2_qs:
kintmp_subsexp(6,2) := cos_beta2_qs:
save kintmp_qs, kintmp_subsexp, sprintf("../codeexport/%s_kinematic_constraints_maple_debug_nach_beta2.m", robot_name):
# Berechne delta3
# Gl. (m3.9)
# Siehe auch: KAS5_m3_sym_codegen_kinematic_constraints_hilfsberechnungen.mw
r_4_D_B := -r_4_G5_D + r_4_G5_B:
t7 := convert_t_s(theta(4)) + delta_qs(16):
# Nutze Additionstheoreme zur Bestimmung von Sinus und Cosinus.
# Dadurch kann Sinus/Cosinus von delta16 direkt genutzt werden und es tritt kein arctan auf.
cos_t7 := cos(convert_t_s(theta(4)))*cos_delta16_qs - sin(convert_t_s(theta(4)))*sin_delta16_qs:
sin_t7 := sin(convert_t_s(theta(4)))*cos_delta16_qs + cos(convert_t_s(theta(4)))*sin_delta16_qs:
# Hilfswinkel t8
t8 := %arctan((sin_t7 * l22 + r_4_D_B(2)), (cos_t7 * l22 - r_4_D_B(1))):
sin_t8 := (sin_t7 * l22 + r_4_D_B(2))/l13:
cos_t8 := (cos_t7 * l22 - r_4_D_B(1))/l13:
delta_qs(3) := t8 - t7:
cos_delta3_qs := (cos_t8)*(cos_t7) + (sin_t8)*(sin_t7):
sin_delta3_qs := (sin_t8)*(cos_t7) - (cos_t8)*(sin_t7):
kintmp_qs(8) := delta_qs(3):
kintmp_subsexp(15,2) := sin_delta3_qs:
kintmp_subsexp(16,2) := cos_delta3_qs:
save kintmp_qs, kintmp_subsexp, sprintf("../codeexport/%s_kinematic_constraints_maple_debug_nach_delta3.m", robot_name):
# Berechne delta8
# Dieser Abschnitt ist der einzige Unterschied zur Berechnung der Parallelkinematik von KAS5_m3.
# Dort wird rho3 aus konstantem delta8 berechnet.
# Hier ist rho3 als verallgemeinerte Koordinate gegeben und das zeitabhängige delta8 wird daraus berechnet
# Gl. (11)
theta3_s := convert_t_s(theta(3)):
delta_qs(8) := theta3_s - gamma3_qs + Pi/2:
# Nutze Additionstheoreme zur Bestimmung von Sinus und Cosinus.
# Dadurch kann Sinus/Cosinus von gamma3 direkt genutzt werden und es tritt kein arctan auf.
cos_delta8_qs := cos(theta3_s+Pi/2)*cos_gamma3_qs + sin(theta3_s+Pi/2)*sin_gamma3_qs:
sin_delta8_qs := sin(theta3_s+Pi/2)*cos_gamma3_qs - cos(theta3_s+Pi/2)*sin_gamma3_qs:
kintmp_qs(13) := delta_qs(8):
kintmp_subsexp(25,2) := sin_delta8_qs:
kintmp_subsexp(26,2) := cos_delta8_qs:
save kintmp_qs, kintmp_subsexp, sprintf("../codeexport/%s_kinematic_constraints_maple_debug_nach_delta8.m", robot_name):
printf("Winkel delta8 berechnet. %s. CPU-Zeit bis hier: %1.2fs.\n", FormatTime("%Y-%m-%d %H:%M:%S"), time()-st):
# Berechne delta6
# Gl. (m3.8)
# Siehe auch: KAS5_m3_sym_codegen_kinematic_constraints_hilfsberechnungen.mw
r_3_G3_E := r_3_G3_D + r_3_D_E:
t1 := -theta3_s + delta_qs(8):
t2 := %arctan((-sin_t1 * l4 + r_3_G3_E(1)), (cos_t1 * l4 + r_3_G3_E(2))):
# Nutze Additionstheoreme zur Bestimmung von Sinus und Cosinus.
cos_t1 := cos(theta3_s)*cos_delta8_qs + sin(theta3_s)*sin_delta8_qs:
sin_t1 := -sin(theta3_s)*cos_delta8_qs + cos(theta3_s)*sin_delta8_qs:
delta_qs(6) := t1 + t2:
kintmp_qs(11) := delta_qs(6):
# Nutze Additionstheoreme zur Bestimmung von Sinus und Cosinus.
cos_t2 := ( cos_t1 * l4 + r_3_G3_E(2)) / l20:
sin_t2 := (-sin_t1 * l4 + r_3_G3_E(1)) / l20:
cos_delta6_qs := cos_t2*cos_t1 - sin_t2*sin_t1:
sin_delta6_qs := sin_t2*cos_t1 + cos_t2*sin_t1:
kintmp_subsexp(21,2) := sin_delta6_qs:
kintmp_subsexp(22,2) := cos_delta6_qs:
save kintmp_qs, kintmp_subsexp, sprintf("../codeexport/%s_kinematic_constraints_maple_debug_nach_delta6.m", robot_name):
# Kompensationsfeder
R_5_6:=rotz(convert_t_s(theta(6))):
r_6_A_H := <l17; 0; 0>:
r_5_A_H:=R_5_6 . r_6_A_H:
r_5_G5_G6:=<l12;0;0>:
r_6_G6_H:=<0; -l18; 0>:
r_5_G6_H:=R_5_6 . r_6_G6_H:
r_5_G5_H:= r_5_G5_G6 + r_5_G6_H: # Weg G5-G6-H
R_12_5_s:=rotz(gamma5_s): # Rotationsmatrix mit allgemeinem zeitabhängigem Ausdruck für den Winkel gamma5
R_12_5 := subs_kintmp_exp(R_12_5_s): # sinus und cosinus von gamma5 direkt ersetzen, ohne den Winkel einzusetzen.
R_5_12:=Transpose(R_12_5):
r_12_B_G5:=<-l14;0;0>:
r_5_G5_B:=R_5_12 . (-r_12_B_G5):
r_5_AB_2 := r_5_A_H - r_5_G5_H + r_5_G5_B: # Weg AB aus A-H-G5-B
l16_c := sqrt(r_5_AB_2[1,1]^2 + r_5_AB_2[2,1]^2): # Länge AB entspricht Federlänge
lpar_qs := l16_c:
printf("Kompensationsfederlänge berechnet. %s.  CPU-Zeit bis hier: %1.2fs.\n", FormatTime("%Y-%m-%d %H:%M:%S"), time()-st):
# Berechne beta1
# Gl. (29)
# Siehe auch: KAS5_m4_sym_codegen_kinematic_constraints_hilfsberechnungen.mw
t2 := cos(convert_t_s(theta(6))):
t1 := sin(convert_t_s(theta(6))):
t3 := %arctan(-(t2 * l18 - sin(gamma5_qs) * l14 + t1 * l17), (t2 * l17 + cos(gamma5_qs) * l14 - t1 * l18 - l12)):
beta1_qs := t3 - gamma5_qs:
cos_t3 := (t2 * l17 + cos(gamma5_qs) * l14 - t1 * l18 - l12) / l16_c:
sin_t3 := (-(t2 * l18 - sin(gamma5_qs) * l14 + t1 * l17)) / l16_c:
kintmp_qs(2) := beta1_qs:
# Nutze Additionstheoreme zur Bestimmung von Sinus und Cosinus.
# Zu erledigen: In den Sinus/Cosinus-Ausdrücken kommt l16 vor. Muss noch generiert werden.
cos_beta1_qs := cos_t3*cos(gamma5_qs) + sin_t3*sin(gamma5_qs):
sin_beta1_qs := sin_t3*cos(gamma5_qs) - cos_t3*sin(gamma5_qs):
kintmp_subsexp(3,2) := sin_beta1_qs:
kintmp_subsexp(4,2) := cos_beta1_qs:
# Exportiere Code für folgende Skripte
# Entferne Annahmen aus Variablen
l1 := 'l1':
l2 := 'l2':
l3 := 'l3':
l4 := 'l4':
l5 := 'l5':
l6 := 'l6':
l7 := 'l7':
l8 := 'l8':
l9 := 'l9':
l10 := 'l10':
l11 := 'l11':
l12 := 'l12':
l13 := 'l13':
l14 := 'l14':
l15 := 'l15':
l16 := 'l16':
l17 := 'l17':
l18 := 'l18':
l19 := 'l19':
l20 := 'l20':
l21 := 'l21':
l22 := 'l22':
# Die Annahmen sind im Ausdruck bereits in den Variablen gespeichert. Ersetze das "~"-Zeichen.
# Quelle: http://www.mapleprimes.com/questions/207601-Remove-Assumptions
nms:=convert(indets(kintmp_qs,name),list): # Liste der Symbole
nmsS:=convert~(nms,string): # Liste der Symbole als String
L:=StringTools:-Substitute~(nmsS,"~",""): #Removing "~"
L1:=parse~(L):
S:=nms=~L1:
kintmp_qs:=subs(S,kintmp_qs):
lpar_qs:=subs(S,lpar_qs):
kintmp_subsexp:=subs(S,kintmp_subsexp):
kintmp_qt := convert_s_t(kintmp_qs):
lpar_qt := convert_s_t(lpar_qs):
# Speichere Maple-Ausdruck (Eingabe-Format und internes Format)
save kintmp_subsexp, sprintf("../codeexport/%s_kinematic_constraints_kintmp_subsexp_maple", robot_name):
save kintmp_subsexp, sprintf("../codeexport/%s_kinematic_constraints_kintmp_subsexp_maple.m", robot_name):
printf("Ausdrücke für kintmp_subsexp gespeichert (Maple). %s. CPU-Zeit bis hier: %1.2fs.\n", FormatTime("%Y-%m-%d %H:%M:%S"), time()-st):
for i from 1 to RowDimension(kintmp_s) do
  tmp := kintmp_qs(i):
  save tmp, sprintf("../codeexport/%s_kinematic_constraints_maple_inert_kintmpq_%d", robot_name, i):
  save tmp, sprintf("../codeexport/%s_kinematic_constraints_maple_inert_kintmpq_%d.m", robot_name, i):
end do:
save kintmp_qs, kintmp_qt, kin_constraints_exist, kintmp_subsexp, sprintf("../codeexport/%s_kinematic_constraints_maple_inert", robot_name):
save kintmp_qs, kintmp_qt, kin_constraints_exist, kintmp_subsexp, sprintf("../codeexport/%s_kinematic_constraints_maple_inert.m", robot_name):
save kintmp_qs, sprintf("../codeexport/%s_kinematic_constraints_kintmp_qs_maple_inert", robot_name):

# Exportieren des vollständigen Ausdruckes
if codegen_act then
  MatlabExport(kintmp_subsexp(..,2), sprintf("../codeexport/KAS5_m4_kinematik_parallel_kintmp_subsexp_matlab.m"), codegen_opt):
end if:
printf("Ausdrücke mit Inert-Arctan exportiert (Matlab). %s. CPU-Zeit bis hier: %1.2fs.\n", FormatTime("%Y-%m-%d %H:%M:%S"), time()-st):
# Liste mit abhängigen konstanten Kinematikparametern erstellen (wichtig für Matlab-Funktionsgenerierung)
read "../helper/proc_list_constant_expressions";
kc_symbols := Matrix(list_constant_expressions( kintmp_subsexp(..,2) )):
save kc_symbols, sprintf("../codeexport/%s_kinematic_constraints_symbols_list_maple", robot_name):
MatlabExport(Transpose(kc_symbols), sprintf("../codeexport/%s_kinematic_constraints_symbols_list_matlab.m", robot_name), 2):
printf("Fertig. %s. CPU-Zeit bis hier: %1.2fs.\n", FormatTime("%Y-%m-%d %H:%M:%S"), time()-st):

