# Base Parameter Properties for Robot based on MDH frames
# Einleitung
# Untersuchung von Eigenschaften der Regressorform und des Minimalparametervektors
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# tree -> Berechnung für eine allgemeine Baumstruktur (aber bisher nur für serielle Strukturen getestet)
# base_parameter_transformations -> Umwandlung der Minimalparameterform in andere Darstellungen
# Authors
# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-04
# 
# Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# Quellen
# [SousaCor2014] Sousa, C. D. and Cortesao, R.: Physical feasibility of robot base inertial parameter identification: A linear matrix inequality approach (2014)
# Initialisierung
interface(warnlevel=0): # Unterdrücke die folgende Warnung.
restart: # Gibt eine Warnung, wenn über Terminal-Maple mit read gestartet wird.
interface(warnlevel=3):
with(LinearAlgebra):
with(ArrayTools):
with(codegen):
with(CodeGeneration):
with(StringTools):
codegen_opt := true:
codegen_act := true:
read "../helper/proc_convert_s_t":
read "../helper/proc_convert_t_s": 
read "../helper/proc_MatlabExport":
read "../robot_codegen_definitions/robot_env":
read sprintf("../codeexport/%s/tree_floatb_definitions", robot_name):
# Ergebnisse der Minimalparametergruppierung laden
if base_method_name="twist" then
  read sprintf("../codeexport/%s/minimal_parameter_vector_fixb_maple", robot_name):
  expstring:="fixb":
  PV2:=Matrix(PV2_vec[11..10*NL,1]):
elif base_method_name="eulangrpy" then 
  read sprintf("../codeexport/%s/minimal_parameter_vector_floatb_eulangrpy_maple", robot_name):
  expstring:="floatb_eulangrpy":
  PV2:=PV2_vec:
else
  printf("Nicht behandelte Basis-Methode: %s\n", base_method_name):
fi:
MPV := Paramvec2:
printf("Generiere Minimalparameter-Transformationen für %s (%s)\n", robot_name, expstring):
# Minimalparametervektor als Matrixdarstellung
# Siehe [SousaCor2014] equ. (38)
# Beispielrechnung:
# U=dU/dPV2*PV2=dU/dMPV*MPV # Parameterlineare Form
# MPV=dMPV/dPV2 * PV2 # Minimalparametervektor MPV hängt nur linear von den normalen Parametern im Parametervektor PV2 ab
# dU/dMPV = dU/dPV2 * dMPV/dPV1
# Benutze Fixed-Base Parameter: Ignoriere Parameter der Basis
Paramvec_size := RowDimension(MPV):
K := Matrix(Paramvec_size, RowDimension(PV2)):
for i to Paramvec_size do 
  for j from 1 to RowDimension(PV2) do 
    K[i,j] := diff(MPV[i, 1], PV2[j,1]):
  end do:
end do:
# Export der Umwandlung von Parametersatz 2 nach Minimalparameter (Matrix)
MatlabExport(K, sprintf("../codeexport/%s/PV2_MPV_transformation_linear_%s_matlab.m", robot_name, expstring), 2):
save K, sprintf("../codeexport/%s/PV2_MPV_transformation_linear_dependant_%s_maple", robot_name, expstring):
# Aufteilung der Matrixdarstellung in einzelne Teilmatrizen
# Generate the Matrices required in [SousaCor2014] equ. (33)
# Wenn ein Inertialparameter zu einer linear unabhängigen Spalte der Regressormatrizen gehört, 
# steht er in K als 1, alle anderen Spalteneinträge dieser Zeile sind Null und der Inertialparameter steht auch in keiner anderen Zeile.
# Falls er zu einer linear abhängigen Spalte der Regressormatrix gehört, steht er in mehreren Spalten von K oder nur in einer Spalte aber dort nicht alleine.
# Die Permutationsmatrizen P_b, P_d wählen die jeweiligen Parameter aus dem Gesamt-Inertialparametervektor aus 
n_b := RowDimension(K):
n_d := ColumnDimension(K)-RowDimension(K): 
n := n_b+n_d: # [SousaCor2014] equ. (31)
P_b := Matrix(n, n_b): # für [SousaCor2014] equ. (33) 
P_d := Matrix(n, n_d): # für [SousaCor2014] equ. (33)
for i from 1 to n_b do:
  # gehe alle Zeilen durch und prüfe, ob der MPV nur aus einem Inertialparameter-Eintrag besteht   
  # falls er nicht aus einem Inertialparameter-Eintrag besteht, kommt der Basis-Eintrag nur in dieser Zeile vor.   
  # falls der MPV-Eintrag aus mehreren Inertialparameter-Einträgen besteht, die nur hier vorkommen, wird der kleinere genommen (der weiter unten steht)

  # gehe alle Spalten durch und prüfe, wenn dieser Eintrag 1 ist, ob er noch in einer anderen Zeile vorkommt.
  linunabh_gefunden :=false: # Merker, ob ein Basis-Inertialparameter gefunden wurde.
  for j from 1 to n do: # alle Spalten durchgehen
    if K(i,j)=1 then
      # printf("In MPV-Zeile %d kommt Inertialparameter %d (%s) direkt vor.\n", i, j, convert(PV2(j,1), string)):
      # gehe alle anderen Zeilen durch
      Index_j_lin_unabh := true: # Merker, dass aktueller Index Basis-Inertialparameter ist.
      for i2 from 1 to n_b do:
        if i = i2 then # die selbe Zeile die gegengeprüft wird ausschließen
          next
        end if:
        if not (K(i2,j) = 0) then
          # Der Inertialparameter kommt auch in einem anderen MPV-Eintrag vor, damit ist er kein Basisparameter
          # printf("Inertialparameter %d (%s) kommt sowohl in MPV-Zeile %d als auch in Zeile %d vor. Kein Basisparameter.\n", j, convert(PV2_vec(10+j,1), string), i, i2):
          Index_j_lin_unabh := false:
          break:
        end if:
      end do:
      if Index_j_lin_unabh then
        # linear unabhängigen Parameter speichern
        P_b(j,i):=1:
        # printf("Für MPV-Zeile %d ist %d (%s) der Basisparameter\n", i, j, convert(PV2(j,1), string)):
        linunabh_gefunden:=true:
      end if:  
    end if:
    if linunabh_gefunden then
      # Nicht weiter nach einem Basisparameter suchen, da bereits einer gefunden wurde.
      break:
    end if
  end do: # j-Schleife über Inertialparameter
  if not linunabh_gefunden then
    # printf("Keinen Basis-Inertialparameter in MPV Zeile %d gefunden.\n", i):
  end if:
end do:
# Alle linear abhängigen Parameter (der Rest) in andere Permutationsmatrix P_d packen
i_d := 0:
for j from 1 to n do:
  # Prüfe ob der betrachtete Parameter in der Permutationsmatrix für linear unabhängige vorkommt
  # Summe über die Zeilen von P_b: Prüfe, ob linear unabhängig
  linunabh := false:
  for i_b from 1 to ColumnDimension(P_b) do:
    if P_b(j, i_b) = 1 then:
      linunabh := true: # kommt in P_b vor: Linear unabhängig!
      break:      
    end if:    
  end do:
  
  if not (linunabh) then 
    i_d := i_d+1; 
    P_d(j, i_d) := 1 
  end if
end do:
# Transformationsmatrix K_d füllen. Setze K_d nur aus den Spalten von K für die linear abhängigen Inertialparameter zusammen
# K_d: Matrix zur Aufteilung des Minimalparametervektors in linear abhängige und unabhängige Inertialparameter
K_d:=Matrix(n_b, n_d):
j_Kd := 0:
for j from 1 to n do:
  # Prüfe, ob der Inertialparameter j linear abhängig ist.
  linabh := false:
  for j_d from 1 to ColumnDimension(P_d) do:
    # j: Index des untersuchten Inertialparameters
    # j_d: Index der untersuchten Spalte von P_d. Ein Eintrag hier bedeutet, dass der Inertialparameter abhängig ist.
    if P_d(j, j_d) = 1 then:
      linabh := true:
      break:      
    end if:    
  end do:

  if linabh then
    j_Kd := j_Kd + 1:
    K_d(..,j_Kd) := K(..,j):
  end if:
end do:
printf("Inertialparametervektor hat %d linear abhängige und %d linear unabhängige Einträge", n_d, n_b):
delta_b := Transpose(P_b) . PV2: # Zur Überprüfung
delta_d := Transpose(P_d) . PV2: # Zur Überprüfung
MatlabExport(K_d, sprintf("../codeexport/%s/PV2_MPV_transformation_linear_dependant_%s_matlab.m", robot_name, expstring), 2):
MatlabExport(P_b, sprintf("../codeexport/%s/PV2_permutation_linear_independant_%s_matlab.m", robot_name, expstring), 2):
MatlabExport(P_d, sprintf("../codeexport/%s/PV2_permutation_linear_dependant_%s_matlab.m", robot_name, expstring), 2):

