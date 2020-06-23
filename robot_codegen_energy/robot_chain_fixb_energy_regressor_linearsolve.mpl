
# Base Parameter Energy Regressor for Robot based on MDH frames
# Einleitung
# Erstellung einer parameterlinearen und -minimalen Regressorform
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# chain -> Berechnung für eine serielle Struktur (nicht: Baumstruktur)
# fixb -> fixed base. Kein Floating base Modell. Dort ist diese Form der Minimalparameterform nicht möglich.
# energy -> Berechnung bezogen auf Energie
# regressor -> Regressorform (parameterlinear)
# linearsolve -> Nicht geometrischer Ansatz nach Khalil, sondern symbolischer Ansatz mit LinearSolve
# Authors
# Jonas Diekmeyer (Studienarbeit bei Elias Knöchelmann)
# Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-06
# 
# (C) Institut fuer Mechatronische Systeme, Leibniz Universitaet Hannover
# Quellen
# [Diekmeyer2018_S678] Identifikation der inversen Dynamik eines seriellen Roboters im geschlossenen Regelkreis, Studienarbeit, imes, LUH, 2018
# Initialisierung
interface(warnlevel=0): # Unterdrücke die folgende Warnung.
restart: # Gibt eine Warnung, wenn über Terminal-Maple mit read gestartet wird.
interface(warnlevel=3):
interface(rtablesize=100): # Zur Anzeige von größeren Vektoren
;
with(LinearAlgebra):
with(ArrayTools):
with(codegen):
with(CodeGeneration):
with(StringTools):
codegen_act := true:
codegen_opt := 2:
read "../helper/proc_convert_s_t":
read "../helper/proc_convert_t_s": 
read "../helper/proc_MatlabExport":
read "../helper/proc_simplify2":
read "../robot_codegen_definitions/robot_env":
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name, base_method_name):
read sprintf("../codeexport/%s/tmp/kinematic_constraints_maple_inert.m", robot_name):  
kin_constraints_exist := kin_constraints_exist: # nur zum Abschätzen der Komplexität
;
# Prüfe, ob die symbolische Berechnung der Parameterminimierung berechnet werden sollte
# Bestimme, ob es eine Baumstruktur ist. Wenn ja, funktioniert der andere Algorithmus nicht und dieser wird genommen.
tree:=false:
for i from 1 to NJ do
  if v(i) ~= i-1 then
    tree := true: break:
  end if:
end do:

if not (assigned(user_CoM) or assigned(user_M) or assigned(user_inertia) \
  or kin_constraints_exist or tree) then
  # es gibt keinen Sonderfall, diese Berechnung der Minimalparameter ist nicht notwendig.
  printf("Keine analytische Berechnung der Minimalparameter notwendig\n"):
  quit: # Funktioniert in GUI nicht richtig...
  robot_name := "": # ...Daher auch Löschung des Roboternamens.
end if:
printf("%s. Generiere Minimalparameterregressor der Energie für %s (symbolischer Ansatz)\n", \ 
  FormatTime("%Y-%m-%d %H:%M:%S"), robot_name, codegen_dynpar):
# Term-Vereinfachungen einstellen
if not assigned(simplify_options) or simplify_options(7)=-1 then # Standard-Einstellungen:
  if not kin_constraints_exist then # normale serielle Ketten und Baumstrukturen
    use_simplify := 0: # Standardmäßig aus
  else # mit kinematischen Zwangsbedingungen
    use_simplify := 1: # standardmäßig simplify-Befehle anwenden
  end if:
else # Benutzer-Einstellungen:
  use_simplify := simplify_options(7): # siebter Eintrag ist für Energie-Regressor
end if:

# Ergebnisse der Energie laden (aus robot_chain_fixb_rotmat_energy_regressor.mw)
read sprintf("../codeexport/%s/tmp/energy_kinetic_fixb_regressor_maple.m", robot_name):
t_ges := t_ges:
read sprintf("../codeexport/%s/tmp/energy_potential_fixb_regressor_maple.m", robot_name):
u_ges := u_ges:
# Parameterminimierung
# Siehe auch [Diekmeyer2018_S678] S. 19f. und 32ff.
# Funktionen definieren

splitSummands := proc(input_expr)
# Gibt eine Liste aller Summanden zurücachezurück
  local expr, sub_expr:

  expr := expand(input_expr):
  
  if type(expr,'`+`') then
    sub_expr := op(expr):
    return sub_expr:
  else
    return expr:
  end if:
  
end proc:

splitFactors := proc(input_expr)
# Gibt eine Liste aller Faktoren zurück
  local sub_expr:
  
  if type(input_expr,'`*`') then
    sub_expr := op(input_expr):
    return sub_expr:
  else
    return input_expr:
  end if:
  
end proc:

removeEmptyRows := proc(h,G)
  local mh, mG, i, j, flag:
  
  flag := Vector[column](Size(G,1)):
  for i from 1 to Size(G,1) do
    flag(i) := 0:
    for j from 1 to Size(G,2) do
      if G(i,j) <> 0 then
        flag(i) := 1:
      end if:
    end do:
  end do:

  mh := Vector[column](Norm(flag,1)):
  mG := Matrix(Norm(flag,1),Size(G,2)):

  j := 1:
  for i from 1 to Size(G,1) do
    if flag(i) = 1 then
      mG(j,..) := G(i,..):
      mh(j) := h(i):
      j := j + 1:
    end if:
  end do:

  return mh, mG:
end proc:

permuteEmptyRows := proc(G)
  local Pb, Pd, i, j, k, flag:
  
  flag := Vector[column](Size(G,1)):
  for i from 1 to Size(G,1) do
    flag(i) := 0:
    for j from 1 to Size(G,2) do
      if G(i,j) <> 0 then
        flag(i) := 1:
      end if:
    end do:
  end do:

  Pb := Matrix(Norm(flag,1),Size(G,1)):
  Pd := Matrix(Size(G,1)-Norm(flag,1),Size(G,1)):

  j := 0:
  k := 0:
  for i from 1 to Size(G,1) do
    if flag(i) = 1 then
      j := j + 1:
      Pb(j,i) := 1:
    elif flag(i) = 0 then
      k := k + 1:
      Pd(k,i) := 1:
    end if:
  end do:

  return Pb, Pd:
end proc:

hasElement := proc(v, expr)
# Gibt den Index des Vektorelements von "v" wieder, das dem Ausdruck "expr" entspricht.
# Ist der Ausdruck nicht enthalten wird der höchste Index um eins erhöht wiedergegeben.
  local i:

  for i from 1 to NumElems(v) do
    if verify(v(i),expr) then
      return i:
    end if:
  end do:
  
  return NumElems(v)+1:
end proc:


# Parameter gruppieren. Siehe [Diekmeyer2018_S678] Gl. 3.27
# Erstelle eine Matrix V, die die einzelnen Terme des Energie-Regressors gruppiert
lagrange := Vector[column](t_ges[1,..] - u_ges[1,..]):

V := Matrix():
w := Vector[column]():
for i from 1 to NumElems(lagrange) do
# printf("i=%d\n", i):
  lagrange_term := simplify(expand(lagrange(i))):
  lagrange_term := convert(lagrange_term,exp):
  lagrange_term := simplify(expand(lagrange_term),size):
  lagrange_summand := [splitSummands(lagrange_term)]:
# print(lagrange_summand):
  for j from 1 to nops(lagrange_summand) do
#   printf("j=%d\n", j):
    # Konstante Ausdrücke werden nicht berücksichtigt!
    if not has(lagrange_summand[j],t) then
      next:
    end if:
    lagrange_factor := [splitFactors(lagrange_summand[j])]:
    lagrange_timeVar := 1:
    lagrange_constant := 1:
    for k from 1 to nops(lagrange_factor) do
#     printf("k=%d\n", k):
      if has(lagrange_factor[k],t) then
        lagrange_timeVar := lagrange_timeVar * lagrange_factor[k]:
      else
        lagrange_constant := lagrange_constant * lagrange_factor[k]:
      end if
    end do:
    lagrange_timeVar := simplify(expand(lagrange_timeVar)):
    index_timeVar := hasElement(w,lagrange_timeVar):
    if index_timeVar > NumElems(w) then
      w(index_timeVar) := lagrange_timeVar:
      V(i,index_timeVar) := lagrange_constant:
    elif i > Size(V,1) then
      V(i,index_timeVar) := lagrange_constant:
    else
      V(i,index_timeVar) := V(i,index_timeVar) + lagrange_constant:
    end if:
  end do:
end do:
# Rang der 
RankV := Rank(V):
printf("%s. Die Komponenten-Matrix des Energie-Regressors hat Rang %d\n", FormatTime("%Y-%m-%d %H:%M:%S"), RankV):

# [Diekmeyer2018_S678] Gl. 3.27
U := LinearSolve(Transpose(V),Transpose(V)):
# Freie Variablen ("_t") entfernen
for i from 1 to Size(V,1) - RankV do
  for j from 1 to Size(V,1) do
    # printf("subs %d,%d\n", i,j):
    U := subs([_t[i, j]=0],U):
  end do:
end do:
# [Diekmeyer2018_S678] Gl. 3.29
Pb, Pd :=permuteEmptyRows(U):
P := Matrix([[Pb],[Pd]]):
# [Diekmeyer2018_S678] Gl. 3.31 (links)
Paramvec2 := Pb.U.PV2_vec(11..,..):
Paramvec2 := simplify2(Paramvec2):
printf("%s. Dimension des Minimalparametervektors: %dx%d\n", \ 
  FormatTime("%Y-%m-%d %H:%M:%S"), RowDimension(Paramvec2), ColumnDimension(Paramvec2)):
Paramvec2;
# [Diekmeyer2018_S678] Gl. 3.31 (rechts)
t_ges_minpar:=Transpose(Pb.Transpose(t_ges)):
u_ges_minpar:=Transpose(Pb.Transpose(u_ges)):
printf("%s. Dimension der Regressormatrix: %dx%d\n", \ 
  FormatTime("%Y-%m-%d %H:%M:%S"), RowDimension(t_ges_minpar), ColumnDimension(t_ges_minpar)):
# Symbolischer Test des Erfolgs der Parameterminimierung: Ist konstant bzw. Null.
# test_u := simplify2(u_ges_minpar.Paramvec2 - u_ges.PV2_vec(11..,..)):
# test_t := simplify2(t_ges_minpar.Paramvec2 - t_ges.PV2_vec(11..,..)):
# Code exportieren
save Paramvec2, sprintf("../codeexport/%s/tmp/minimal_parameter_vector_fixb_maple", robot_name):
save Paramvec2, sprintf("../codeexport/%s/tmp/minimal_parameter_vector_fixb_maple_linsolve", robot_name): # zum Testen gegen andere Implementierung
if codegen_act then
   MatlabExport(Paramvec2, sprintf("../codeexport/%s/tmp/minimal_parameter_vector_fixb_matlab.m", robot_name), codegen_opt):
end if;
save t_ges_minpar, sprintf("../codeexport/%s/tmp/energy_kinetic_fixb_regressor_minpar_maple.m", robot_name):
save u_ges_minpar, sprintf("../codeexport/%s/tmp/energy_potential_fixb_regressor_minpar_maple.m", robot_name):
if codegen_act then
  MatlabExport(convert_t_s(t_ges_minpar), sprintf("../codeexport/%s/tmp/energy_kinetic_fixb_regressor_minpar_matlab.m", robot_name), codegen_opt):
end if:
if codegen_act then
  MatlabExport(convert_t_s(u_ges_minpar), sprintf("../codeexport/%s/tmp/energy_potential_fixb_regressor_minpar_matlab.m", robot_name), codegen_opt):
end if:


