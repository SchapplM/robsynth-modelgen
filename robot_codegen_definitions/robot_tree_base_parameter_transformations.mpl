# Base Parameter Properties for Robot based on MDH frames
# Einleitung
# Untersuchung von Eigenschaften der Regressorform und des Minimalparametervektors
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# tree -> Berechnung für eine allgemeine Baumstruktur
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
printf("Generiere Minimalparameterregressor der Energie für %s\n", robot_name, codegen_dynpar):
read sprintf("../codeexport/%s/tree_floatb_twist_definitions", robot_name):
# Ergebnisse der Minimalparametergruppierung laden
read sprintf("../codeexport/%s/minimal_parameter_vector_fixb_maple", robot_name):
MPV_fixb := Paramvec2:
# Minimalparametervektor als Matrixdarstellung
# Siehe [SousaCor2014] equ. (38)
# Beispielrechnung:
# U=dU/dPV2*PV2=dU/dMPV*MPV # Parameterlineare Form
# MPV=dMPV/dPV2 * PV2 # Minimalparametervektor MPV hängt nur linear von den normalen Parametern im Parametervektor PV2 ab
# dU/dMPV = dU/dPV2 * dMPV/dPV1
# Benutze Fixed-Base Parameter: Ignoriere Parameter der Basis
Paramvec_size := RowDimension(MPV_fixb):
dMPVdPV2 := Matrix(Paramvec_size, 10*NJ):
for i to Paramvec_size do 
  for j from 1 to 10*NJ do 
    dMPVdPV2[i, j] := diff(MPV_fixb[i, 1], PV2_vec[10+j, 1]):
  end do:
end do:
# Export der Umwandlung von Parametersatz 2 nach Minimalparameter (Matrix)
MatlabExport(dMPVdPV2, sprintf("../codeexport/%s/PV2_MPV_transformation_linear_fixb_matlab.m", robot_name), 2):
save dMPVdPV2, sprintf("../codeexport/%s/PV2_MPV_transformation_linear_dependant_fixb_maple", robot_name):

# Aufteilung der Matrixdarstellung in einzelne Teilmatrizen
K := dMPVdPV2:
# Generate the Matrices required in [SousaCor2014] equ. (33)
# Wenn ein Inertialparameter zu einer linear unabhängigen Spalte der Regressormatrizen gehört, 
# steht er in dX2dX1 als 1, alle anderen SpalteneintrÃ¤ge dieser Zeile sind Null und der Inertialparameter steht auch in keiner anderen Zeile.
# Falls er zu einer linear abhängigen Spalte der Regressormatrix gehört, steht er in mehreren Spalten von dX2dX1
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
      printf("In MPV-Zeile %d kommt Inertialparameter %d (%s) direkt vor.\n", i, j, convert(PV2_vec(10+j,1), string)):
      # gehe alle anderen Zeilen durch
      Index_j_lin_unabh := true: # Merker, dass aktueller Index Basis-Inertialparameter ist.
      for i2 from 1 to n_b do:
        if i = i2 then # die selbe Zeile die gegengeprüft wird ausschließen
          next
        end if:
        if not (K(i2,j) = 0) then
          # Der Inertialparameter kommt auch in einem anderen MPV-Eintrag vor, damit ist er kein Basisparameter
          printf("Inertialparameter %d (%s) kommt sowohl in MPV-Zeile %d als auch in Zeile %d vor. Kein Basisparameter.\n", j, convert(PV2_vec(10+j,1), string), i, i2):
          Index_j_lin_unabh := false:
          break:
        end if:
      end do:
      if Index_j_lin_unabh then
        # linear unabhängigen Parameter speichern
        P_b(j,i):=1:
        printf("Für MPV-Zeile %d ist %d (%s) der Basisparameter\n", i, j, convert(PV2_vec(10+j,1), string)):
        linunabh_gefunden:=true:
      end if:  
    end if:
    if linunabh_gefunden then
      # Nicht weiter nach einem Basisparameter suchen, da bereits einer gefunden wurde.
      break:
    end if
  end do: # j-Schleife über Inertialparameter
  if not linunabh_gefunden then
    printf("Keinen Basis-Inertialparameter in MPV Zeile %d gefunden.\n", i):
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
MatlabExport(K_d, sprintf("../codeexport/%s/PV2_MPV_transformation_linear_dependant_matlab.m", robot_name), 2):
MatlabExport(P_b, sprintf("../codeexport/%s/PV2_permutation_linear_independant_matlab.m", robot_name), 2):
MatlabExport(P_d, sprintf("../codeexport/%s/PV2_permutation_linear_dependant_matlab.m", robot_name), 2):
