
# Berechnungen für Systeme mit impliziten kinematischen Zwangsbedingungen
# Beschreibung
# Allgemeine Berechnungen zu den impliziten Zwangsbedingungen
# Die Bedingungen müssen vorher in einer Roboterspezifischen Datei berechnet werden (händische Erstellung der kinematischen Schleifen und sonstigen Zwangsbedingungen
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# kinematic_constraints_calculations_implicit -> Berechnungen bezüglich implizit definierter kinematischer Zwangsbedingungen
# Quellen
# [Docquier2013] Docquier, Nicolas and Poncelet, Antoine and Fisette, Paul: ROBOTRAN: a powerful symbolic gnerator of multibody models (2013)
# [DoThanhKotHeiOrt2009b] Do Thanh et al.: On the inverse dynamics problem of general parallel robots (2009)
# [ParkChoPlo1999] Park, FC and Choi, Jihyeon and Ploen, SR: Symbolic formulation of closed chain dynamics in independent coordinates
# Autor
# Moritz Schappler, schappler@imes.uni-hannover.de, 2018-02
# Institut fuer Mechatronische Systeme, Leibniz Universitaet Hannover
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
read "../helper/proc_convert_s_t":
read "../helper/proc_convert_t_s":
with(RealDomain): # Schränkt alle Funktionen auf den reellen Bereich ein. Muss nach Definition von MatlabExport kommen. Sonst geht dieses nicht.
;
read "../robot_codegen_constraints/proc_subs_kintmp_exp":
# Definition und Zwangsbedingeun
read "../robot_codegen_definitions/robot_env":
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name):
# Lesen der Zwangsbedingungen
kin_constraints_exist := false:
constrfile := sprintf("../codeexport/%s/tmp/kinematic_constraints_implicit_maple.m", robot_name):
if FileTools[Exists](constrfile) then
  read constrfile:
  implconstr_s := implconstr_s: # Für Sichtbarkeit der Variablen
  implconstr_t := implconstr_t: # Für Sichtbarkeit der Variablen
  kin_constraints_exist := kin_constraints_exist: # Für Sichtbarkeit der Variablen
end if:

if kin_constraints_exist = true then:
  printf("Kinematische Zwangsbedingungen in impliziter Form gelesen.\n"):
else
  printf("Es gibt keine impliziten Zwangsbedingungen. Offene Struktur oder nur explizit definierte Bedingungen. Keine weiteren Berechnungen notwendig."):
  restart: # Funktioniert nicht. Gedacht, damit im worksheet-Modus nichts mehr passieren kann.
  robot_name := "": # Damit werden die Export-Befehle ungültig
  codegen_act := false:
  quit:
end if:
NIZB := RowDimension(implconstr_s):
# Gelenkdefinitionen
# Setze aktuierte Gelenke als Minimalkoordinaten voraus
NAJ := add(mu(k), k=1..NJ):
NPJ := NQJ-NAJ:
# Bestimme Indizes der aktiven und passiven Gelenke
# Entspricht Partitionierung der Gelenkwinkel in q1 und q2 in [Docquier2013]. Hier kann die Reihenfolge von aktiven und passiven Winkeln auch durchmischt sein.
IndAct := Matrix(NAJ, 1):
IndPass:= Matrix(NPJ, 1):
ka := 1: kp := 1:
for i from 1 to NQJ do
  if mu(i) = 1 then
    IndAct(ka) := i:
    ka := ka + 1:
  else
    IndPass(kp) := i:
    kp := kp + 1:
  end if:
end do:
printf("Indizes der %d aktiven Gelenke:\n", NAJ);
Transpose(IndAct);
printf("Indizes der %d passiven Gelenke:\n", NPJ);
Transpose(IndPass);
printf("%d Zwangsbedingungsgleichungen\n", NIZB);
# Jacobi-Matrix der Impliziten Zwangsbedingungen in Abhängigkeit der unabhängigen Koordinaten
# 
# Entspricht J1 in [Docquier2013], A in [ParkChoPlo1999]
Phia_s := Matrix(NIZB, NAJ):
# Jacobi-Matrix berechnen
for i from 1 to NIZB do
  for j from 1 to NAJ do
    k := IndAct(j):
    Phia_s(i,j) := diff(implconstr_s(i,1), qJ_s(k,1)):
  end do:
end do:
# Jacobi-Matrix der Impliziten Zwangsbedingungen in Abhängigkeit der abhängigen Koordinaten
# 
# Entspricht J2 in [Docquier2013], P in [ParkChoPlo1999]
Phip_s := Matrix(NIZB, NPJ):
# Jacobi-Matrix berechnen
for i from 1 to NIZB do
  for j from 1 to NPJ do
    k := IndPass(j):
    Phip_s(i,j) := diff(implconstr_s(i,1), qJ_s(k,1)):
  end do:
end do:
# Zeitableitung der Jacobi-Matrix
# Wird benötigt, um die Beschleunigung der Gelenkkoordinaten der offenen Struktur zu berechnen.
Phia_t := convert_s_t(Phia_s):
PhiaD_t := diff~(Phia_t, t):
PhiaD_s := convert_t_s(PhiaD_t):

Phip_t := convert_s_t(Phip_s):
PhipD_t := diff~(Phip_t, t):
PhipD_s := convert_t_s(PhipD_t):

# Exportiere Code für folgende Skripte
# Speichere Maple-Ausdruck (Eingabe-Format und internes Format)
save Phia_s, sprintf("../codeexport/%s/tmp/kinematic_constraints_explicit_active_jacobian_maple.m", robot_name):
save PhiaD_s, sprintf("../codeexport/%s/tmp/kinematic_constraints_explicit_active_jacobian_time_derivative_maple.m", robot_name):
save Phip_s, sprintf("../codeexport/%s/tmp/kinematic_constraints_explicit_passive_jacobian_maple.m", robot_name):
save PhipD_s, sprintf("../codeexport/%s/tmp/kinematic_constraints_explicit_passive_jacobian_time_derivative_maple.m", robot_name):
printf("Ausdrücke für Kinematische ZB gespeichert (Maple)\n"):
if codegen_act then
  MatlabExport(implconstr_s, sprintf("../codeexport/%s/tmp/kinconstr_impl_matlab.m", robot_name), codegen_opt):
  MatlabExport(Phia_s, sprintf("../codeexport/%s/tmp/kinconstr_impl_active_jacobian_matlab.m", robot_name), codegen_opt):
  MatlabExport(PhiaD_s, sprintf("../codeexport/%s/tmp/kinconstr_impl_active_jacobianD_matlab.m", robot_name), codegen_opt):
  MatlabExport(Phip_s, sprintf("../codeexport/%s/tmp/kinconstr_impl_passive_jacobian_matlab.m", robot_name), codegen_opt):
  MatlabExport(PhipD_s, sprintf("../codeexport/%s/tmp/kinconstr_impl_passive_jacobianD_matlab.m", robot_name), codegen_opt):
  printf("Ausdrücke für Kinematische ZB gespeichert (Matlab)\n"):
end if:

# Invertierung der Gradientenmatrix
# Invertiere die Gradientenmatrix bezüglich der passiven Gelenke
n1 := RowDimension(Phip_s):
n2 := ColumnDimension(Phip_s):
# Erstelle Matrix ppf mit der Form der Gradientmatrix Phip_s
ppf := Matrix(n1,n2,symbol=uu):
for i from 1 to n1 do
  for j from 1 to n2 do
    if Phip_s(i,j) = 0 then
      ppf(i,j) := 0:
    end if:
  end do:
end do:
# Invertiere die allgemeine Form der Matrix und setze dann die Einträge ein.
InvPhip_sf := MatrixInverse(ppf):
InvPhip_s := InvPhip_sf:
for i from 1 to n1 do
  for j from 1 to n2 do
    if not Phip_s(i,j) = 0 then
      InvPhip_s := subs({ppf(i,j)=Phip_s(i,j)}, InvPhip_s):
    end if:
  end do:
end do:
save InvPhip_s, sprintf("../codeexport/%s/tmp/kinematic_constraints_explicit_passive_jacobian_inv_maple", robot_name):
if codegen_act then
  MatlabExport(InvPhip_sf, sprintf("../codeexport/%s/tmp/kinconstr_impl_passive_jacobian_inv_matlab.m", robot_name), codegen_opt):
end if:
# Projektionsmatrix aus impliziter Gradientenmatrix
# 
# Form der Projektionsmatrix anschauen
B21f := -InvPhip_sf.Phia_s:
# Projektionsmatrix berechnen. [Docquier2013], Text nach Gl. 12
B21 := -InvPhip_s.Phia_s:
save B21, sprintf("../codeexport/%s/tmp/kinconstr_impl_projection_jacobian_maple", robot_name):
if codegen_act then
  MatlabExport(B21, sprintf("../codeexport/%s/tmp/kinconstr_impl_projection_jacobian_matlab.m", robot_name), codegen_opt):
end if:
# Zeitableitung der Projektionsmatrix
# [ParkChoPlo1999], Text nach Gl. 73
# [DoThanhKotHeiOrt2009b], Gl. 20
B21D := (InvPhip_s . PhipD_s . InvPhip_s . Phia_s ) + (-InvPhip_s.PhiaD_s):
save B21D, sprintf("../codeexport/%s/tmp/kinconstr_impl_projection_jacobian_derivative_maple", robot_name):
if codegen_act then
  MatlabExport(B21D, sprintf("../codeexport/%s/tmp/kinconstr_impl_projection_jacobian_derivative_matlab.m", robot_name), codegen_opt):
end if:

printf("Fertig\n"):

