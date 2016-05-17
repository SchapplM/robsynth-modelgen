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
# Calculate Forward Kinematics
# Trf is the Matrix of Transformation from i-1 to i
# Trf_c is the cummulated Matrix of Transformation from 0 to i
Trf_c := Matrix(4, 4, NL): # Diese Initialisierung bringt nichts (initialisiert nur 4x4-Matrix)
Trf := Matrix(4, 4, NJ): # Diese Initialisierung bringt nichts (initialisiert nur 4x4-Matrix)
for i from 1 to NJ do 
  Trf(1 .. 4, 1 .. 4, i) := Matrix(4,4):
end do:
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
for i from 1 to NJ do 
  Trf(1 .. 4, 1 .. 4, i) := trafo_mdh_full(alpha[i,1], a[i,1], theta[i,1], d[i,1], beta[i,1], b[i,1]):
  # Index des vorherigen Koordinatensystems
  j := v(i)+1:
  Trf_c(1 .. 4, 1 .. 4, i+1) := Multiply(Trf_c(1 .. 4, 1 .. 4, j), Trf(1 .. 4, 1 .. 4, i)):
end do:
# Export
# Maple-Export
save Trf, Trf_c, sprintf("../codeexport/%s_kinematics_floatb_%s_rotmat_maple.m", robot_name, base_method_name):
save Trf, Trf_c, sprintf("../codeexport/%s_kinematics_floatb_%s_rotmat_maple", robot_name, base_method_name):
# Export des symbolischen Ausdrucks für alle Transformationsmatrizen auf einmal.
Trf_c_Export := Matrix((NL)*4, 4):
for i from 1 to NL do 
  Trf_c_Export((i-1)*4+1 .. 4*i, 1..4) := Trf_c(1..4, 1..4, i):
end do:
if codegen_act then
  MatlabExport(convert_t_s(Trf_c_Export), sprintf("../codeexport/%s_fkine_floatb_%s_rotmat_matlab.m", robot_name, base_method_name), codegen_opt):
end if:
# Export des symbolischen Ausdrucks für jede Transformationsmatrix einzeln
for i from 1 to NL do
  if codegen_act then
    MatlabExport(convert_t_s(Trf_c(1 .. 4, 1 .. 4, i)), sprintf("../codeexport/%s_fkine_%d_floatb_%s_rotmat_matlab.m", robot_name, i, base_method_name), codegen_opt):
  end if:
end do:

