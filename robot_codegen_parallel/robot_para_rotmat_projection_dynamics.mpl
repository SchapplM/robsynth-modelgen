
# Berechnung und Projektion der Dynamikgleichungen
# Einleitung
# Berechnung und Projektion der Dynamikgleichungen
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# para -> Berechnung für eine parallelen Roboter
# rotmat -> Kinematik wird mit Rotationsmatrizen berechnet
# projection -> Die Dynamikgleichungen werden auf EE-Koordinaten projiziert
# kinematics -> Berechnung der Kinematik
# Autor
# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# Sources
# [GautierKhalil1990] Direct Calculation of Minimum Set of Inertial Parameters of Serial Robots
# [KhalilDombre2002] Modeling, Identification and Control of Robots
# [Ortmaier2014] Vorlesungsskript Robotik I
# Initialization
#interface(warnlevel=0): # Unterdrücke die folgende Warnung.
restart: # Gibt eine Warnung, wenn über Terminal-Maple mit read gestartet wird.
#interface(warnlevel=3):
with(LinearAlgebra):
#with(ArrayTools):
with(codegen):
with(CodeGeneration):
with(StringTools):
# Einstellungen für Code-Export: Optimierungsgrad (2=höchster) und Aktivierung jedes Terms.
#codegen_act := true: # noch nicht implementiert
codegen_debug := false:
codegen_opt := 2:
codeexport_invdyn := true:
read "../helper/proc_convert_s_t":
read "../helper/proc_convert_t_s": 
read "../helper/proc_MatlabExport":
read "../helper/proc_index_symmat2vector":
read "../helper/proc_symmat2vector":
read "../helper/proc_vec2skew":
read "../helper/proc_skew2vec":
read "../transformation/proc_rotx": 
read "../transformation/proc_roty": 
read "../transformation/proc_rotz": 
read "../transformation/proc_trotx": 
read "../transformation/proc_troty": 
read "../transformation/proc_trotz": 
read "../transformation/proc_transl": 
read "../transformation/proc_trafo_mdh": 
read "../robot_codegen_definitions/robot_env":
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name):
# Kennung des Parametersatzes, für den die Dynamikfunktionen erstellt werden sollen. Muss im Repo und in der mpl-Datei auf 1 gelassen werden, da die folgende Zeile mit einem Skript verarbeitet wird.
codegen_dynpar := 1:
# Link-Index, für den die Jacobi-Matrix aufgestellt wird. Hier wird angenommen, dass der Endeffektor das letzte Segment (=Link) ist. Die Jacobi-Matrix kann hier aber für beliebige Segmente aufgestellt werden. (0=Basis)
LIJAC:=NL-1:
# Ergebnisse der Plattform-Dynamik laden
read sprintf("../codeexport/%s/tmp/floatb_%s_platform_dynamic_maple.m", robot_name, base_method_name):
# Ergebnisse der Dynamik der Gelenkkette laden
read sprintf("../codeexport/%s/tmp/invdyn_floatb_%s_par%d_maple.m", robot_name, base_method_name, codegen_dynpar)
;
# Ergebnisse der Kinematik für parallelen Roboter laden
read sprintf("../codeexport/%s/tmp/kinematics_%s_platform_maple.m", robot_name, base_method_name):
# Ergebnisse G-Vektor laden
read sprintf("../codeexport/%s/tmp/gravload_par%d_maple.m", robot_name, codegen_dynpar):
G := Matrix(taug_s(7..NQ,1)):
# Ergebnisse C-Vektor laden
read sprintf("../codeexport/%s/tmp/coriolisvec_par%d_maple.m", robot_name, codegen_dynpar):
Cvec := Matrix(tauCC_s(7..NQ,1)):
# Ergebnisse M-Matrix laden
read sprintf("../codeexport/%s/tmp/inertia_par%d_maple.m", robot_name, codegen_dynpar):
MM := MM_s(7..NQ,7..NQ):
# Ergebnisse der Kinematik für parallen Roboter laden
read sprintf("../codeexport/%s/tmp/kinematics_%s_platform_maple.m", robot_name, base_method_name):
printf("Generiere Dynamik für PKM %s mit Parametersatz %d und %s\n", robot_name, codegen_dynpar, base_method_name):
# Berechne Dynamik-Matrizen für alle Beine
# Dupliziere alle berechneten Matrizen. i steht für den Index des jeweiligen Beines
for i to N_LEGS do
  MM||i := Copy(MM):
  Cvec||i := Copy(Cvec):
  G||i := Copy(G):
end do:
# Substituiere in jeder Matrix den Winkel Alpha (Verdrehung in der Basis) und die Gelenkkoordinaten und -geschwindigkeiten

for k from 1 by 1 to N_LEGS do
  for i to NQJ do
  	Cvec||k(i,1):=subs({alpha=alpha[k]},Cvec||k(i,1)):
  	G||k(i,1):=subs({alpha=alpha[k]},G||k(i,1)):
    for m to NQJ do #alpha
      n := (m + (k-1)*NQJ):
      Cvec||k(i,1):=subs({qJD||m||s=qJ||D||n||s,qJ||m||s=qJ||n||s},Cvec||k(i,1)):
      G||k(i,1):=subs({qJ||m||s=qJ||n||s},G||k(i,1)):
    end do:
    for j to NQJ do
    	 MM||k(i,j):=subs({alpha=alpha[k]},MM||k(i,j)):
      for m to NQJ do #alpha
        n := m + (k-1)*NQJ:
        MM||k(i,j):=subs({qJ||m||s=qJ||n||s},MM||k(i,j)):
      end do:
    end do:
  end do:
end do:

# Berechnung, Projektion und Addition der Dynamikgleichungen
# Berechnung der Kräfte/Momente an den Gelenken der jeweiligen Beine und Projektion auf EE-Plattform
for i to N_LEGS do
  k := NQJ*(i-1):
  A||i := Multiply(JBinv_i(..,..,i),JBD_i(..,..,i)):
  B||i := Multiply(-MM||i,Multiply(A||i,<qJD||(1+k)||s;qJD||(2+k)||s;qJD||(3+k)||s>)):
  tau||i := Transpose(U_i(..,..,i)).Transpose(JBinv_i(..,..,i)).MM||i.JBinv_i(..,..,i).(U_i(..,..,i).H.xEDD_s+U_i(..,..,i).dH.xED_s+UD_i(..,..,i).H.xED_s) + Multiply(Transpose(U_i(..,..,i)),Multiply(Transpose(JBinv_i(..,..,i)),(B||i+Cvec||i+G||i))):
end do:
# Aufsummieren aller Kräfte, projiziert auf EE-Plattform
Tmp := 0:
for i to N_LEGS do
  Tmp := Tmp + tau||i:
end do:
# Addiere Inverse Dynamik der Plattform
tauGes := Tmp + tauE:
# Replace Joint Velocities
# Substituiere die Gelenkgeschwindigkeiten über H-, Ui- und JBi-Matrix mit EE-Geschwindikeiten
Tmp := 0:
for i to N_LEGS do
  Tmp := Multiply(H,xED_s):
  Tmp := Multiply(U_i(..,..,i),Tmp):
  z||i := Multiply(JBinv_i(..,..,i),Tmp):
end do:
for i to 6 do
  for j to N_LEGS do
    k := NQJ*(j-1):
    tauGes(i,1) := subs({qJD||(1+k)||s=z||j(1),qJD||(2+k)||s=z||j(2),qJD||(3+k)||s=z||j(3)},tauGes(i,1)):
  end do:
end do:
# Export
tau := tauGes:
# Matlab Export: Floating base
# Berechnung der Basis-Belastung ist für manche Basis-Darstellungen falsch (siehe oben unter Gravitationslast).
if codeexport_invdyn then
  MatlabExport(tau, sprintf("../codeexport/%s/tmp/invdyn_para_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:

