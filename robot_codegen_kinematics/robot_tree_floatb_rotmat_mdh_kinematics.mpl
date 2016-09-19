# Forward Kinematics for Robot based on MDH frames
# Introduction
# Direkte Kinematik basierend auf MDH-Parametern berechnen
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# tree -> Berechnung für eine beliebige Baumstruktur (ohne Schleifen)
# floatb -> floating base wird durch base twist (Geschwindigkeit der Basis) oder vollständige Orientierung (Euler-Winkel) berücksichtigt
# rotmat -> Kinematik wird mit Rotationsmatrizen berechnet
# mdh_kinematics -> Berechnung der Vorwärtskinematik mit modifizierten DH-Parametern nach [KhalilKle1986]
# 
# Prinzip:
# Berechne die direkte Kinematik.
# Zusätzlich können kinematische Zwangsbedingungen direkt berücksichtigt werden. Diese sorgen dafür, dass MDH-Winkel durch analytische Ausdrücke verallgemeinerter Koordinaten ersetzt werden.
# Authors
# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# Sources
# [KhalilKle1986] Khalil, W. & Kleinfinger, J.: A new geometric notation for open and closed-loop robots (1986)
# [KhalilDombre2002] Modeling, Identification and Control of Robots
# [Ortmaier2014] Vorlesungsskript Robotik I
# Initialization
restart:
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
read "../transformation/proc_rotx": 
read "../transformation/proc_roty": 
read "../transformation/proc_rotz": 
read "../transformation/proc_trotx": 
read "../transformation/proc_troty": 
read "../transformation/proc_trotz": 
read "../transformation/proc_transl": 
read "../transformation/proc_trafo_mdh": 
read "../transformation/proc_rpy2r": 
read "../transformation/proc_rpy2tr": 
read "../robot_codegen_definitions/robot_env":
printf("Generiere Kinematik für %s\n", robot_name):
read sprintf("../codeexport/%s_tree_floatb_definitions", robot_name):
# Kinematische Zwangsbedingungen
# Zwangsbedingungen für m3 und m4 gleich.
# Lese Variablen: kintmp_subsexp, kintmp_qs, kintmp_qt
kin_constraints_exist := false:
read "../robot_codegen_constraints/proc_subs_kintmp_exp":
read sprintf("../codeexport/%s_kinematic_constraints_maple_inert.m", robot_name):
if kin_constraints_exist = true then:
  kintmp_qs := kintmp_qs: # gelesene Variable sonst nicht sichtbar
  kintmp_qt := kintmp_qt: # gelesene Variable sonst nicht sichtbar
  kintmp_subsexp := kintmp_subsexp:
  printf("Kinematische Zwangsbedingungen gelesen."):
else
  kintmp_t := Matrix(1,1):
  kintmp_s := Matrix(1,1):
  kintmp_qs := Matrix(1,1):
  kintmp_qt := Matrix(1,1):
  kintmp_subsexp := Matrix(1,2):
end if:
# Calculate Forward Kinematics (Single-Joint Transformation)
# Trf is the Matrix of Transformation from i-1 to i
# Trf_c is the cummulated Matrix of Transformation from 0 to i
Trf := Matrix(4, 4, NL-1): # Diese Initialisierung bringt nichts (initialisiert nur 4x4-Matrix)
for i from 1 to NL-1 do 
  Trf(1 .. 4, 1 .. 4, i) := Matrix(4,4):
end do:
for i from 1 to NL-1 do 
  Trf(1 .. 4, 1 .. 4, i) := trafo_mdh_full(alpha[i,1], a[i,1], theta[i,1], d[i,1], beta[i,1], b[i,1]):
end do:
# Kinematische Zwangsbedingungen ersetzen
# Substituiere allgemeine Ausdrücke der Winkel der Parallelstruktur mit kinematischen Zwangsbedingungen in Abhängigkeit der Haupt-Gelenkwinkel
for i from 1 to NL-1 do # Index über Transformationsmatrizen aller Körper
  for ix from 1 to 4 do # Index über Zeilen der Transformationsmatrizen
    for iy from 1 to 4 do # Index über Spalten der Transformationsmatrizen
      # Substituiere Zeitvariablen
      Trf(ix, iy, i) := convert_t_s( Trf(ix, iy, i) ):
      # Substituiere sin und cos der Winkel (einfachere Ausdrücke)
      Trf(ix, iy, i) := subs_kintmp_exp(Trf(ix, iy, i)):
      # Substituiere die verbleibenden Winkel direkt (einige Winkel sind nicht in den Ersetzungsausdrücken enthalten, da sie keine problematischen arctan-Ausdrücke enthalten.
      for jj from 1 to RowDimension(kintmp_qt) do # Index über zu ersetzende Winkel
        Trf(ix, iy, i) := subs( { kintmp_t(jj, 1) = kintmp_qt(jj, 1) }, Trf(ix, iy, i) ): 
      end do:
      Trf(ix, iy, i) := convert_s_t( Trf(ix, iy, i) ):
    end do:
  end do:
end do:
if kin_constraints_exist = true then:
  printf("Ersetzungen der MDH-Parameter mit Ergebnissen der Parallelstruktur in verallgemeinerten Koordinaten erfolgreich."):
end if:

# Calculate Forward Kinematics (Multi-Joint Transformation)
Trf_c := Matrix(4, 4, NJ): # Diese Initialisierung bringt nichts (initialisiert nur 4x4-Matrix)
for i from 1 to NL do 
  Trf_c(1 .. 4, 1 .. 4, i) := Matrix(4,4):
end do:
# Basis-Transformation: Unterschiedliche Darstellungsmethoden. Führen zu unterschiedlichen verallgemeinerten Koordinaten.
if base_method_name = "twist" then:
  Trf_c(1 .. 4, 1 .. 4, 1) := transl(X_base_t[1..3,1]):
end:
if base_method_name = "eulangrpy" then:
  Trf_c(1 .. 4, 1 .. 4, 1) := transl(X_base_t[1..3,1]).rpy2tr(X_base_t[4,1], X_base_t[5,1], X_base_t[6,1]):
end:
printf("Nutze die Methode %s für die Basis-Orientierung\n", base_method_name):
# Kinematik aller Körper mit MDH-Ansatz Bestimmen. [KhalilKle1986].
for i from 1 to NL-1 do 
  # Index des vorherigen Koordinatensystems
  j := v(i)+1:
  Trf_c(1 .. 4, 1 .. 4, i+1) := Multiply(Trf_c(1 .. 4, 1 .. 4, j), Trf(1 .. 4, 1 .. 4, i)):
end do:

# Export
# Maple-Export
save Trf, Trf_c, sprintf("../codeexport/%s_kinematics_floatb_%s_rotmat_maple.m", robot_name, base_method_name):
save Trf, Trf_c, sprintf("../codeexport/%s_kinematics_floatb_%s_rotmat_maple", robot_name, base_method_name):
# Export des symbolischen Ausdrucks für alle kumulierten Transformationsmatrizen auf einmal.
Trf_c_Export := Matrix((NL)*4, 4):
for i from 1 to NL do 
  Trf_c_Export((i-1)*4+1 .. 4*i, 1..4) := Trf_c(1..4, 1..4, i):
end do:
if codegen_act then
  MatlabExport(convert_t_s(Trf_c_Export), sprintf("../codeexport/%s_fkine_mdh_floatb_%s_rotmat_matlab.m", robot_name, base_method_name), codegen_opt):
end if:
# Export des symbolischen Ausdrucks für alle Gelenk-Transformationsmatrizen auf einmal.
Trf_Export := Matrix((NL-1)*4, 4):
for i from 1 to (NL-1) do 
  Trf_Export((i-1)*4+1 .. 4*i, 1..4) := Trf(1..4, 1..4, i):
end do:
if codegen_act then
  MatlabExport(convert_t_s(Trf_Export), sprintf("../codeexport/%s_joint_transformation_mdh_rotmat_matlab.m", robot_name), codegen_opt):
end if:
# Export des symbolischen Ausdrucks für jede Transformationsmatrix einzeln
for i from 1 to NL do
  if codegen_act then
    MatlabExport(convert_t_s(Trf_c(1 .. 4, 1 .. 4, i)), sprintf("../codeexport/%s_fkine_%d_floatb_%s_rotmat_matlab.m", robot_name, i, base_method_name), codegen_opt):
  end if:
end do:

