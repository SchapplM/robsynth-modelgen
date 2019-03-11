
# Definitions for Robot Dynamics Code Generation
# Init
# Erstelle Definitionen für die Maple-Skripte zur Berechnung von Kinematik und Dynamik des Roboters
# Nutze Euler-Winkel (RPY-Konvention) zur Darstellung der Basisorientierung
# 
# Quellen:
# [1] Sousa, C. D. and Cortesao, R.: Physical feasibility of robot base inertial parameter identification: A linear matrix inequality approach (2014)
# [2] Ayusawa, K. and Venture, G. and Nakamura, Y.: Identifiability and identification of inertial parameters using the underactuated base-link dynamics for legged multibody systems (2013)
# [3] Fujimoto, Y. and Obata, S. and Kawamura, A.: Robust biped walking with active interaction control between foot and ground (1998)
# [4] Khalil, W. and Kleinfinger, J.-F.: Minimum operations and minimum parameters of the dynamic models of tree structure robots (1987)
# [5] Ortmaier: Robotik I Vorlesungsskript
# 
# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-04
# (C) Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
interface(warnlevel=0): # Unterdrücke die folgende Warnung.
restart: # Gibt eine Warnung, wenn über Terminal-Maple mit read gestartet wird.
interface(warnlevel=3):
# Funktion für Euler-Transformation aus Robotik-Repo laden (Pfad muss bekannt sein)
read("../robotics_repo_path"):
read(sprintf("%s/transformation/maple/proc_eulxyzjac", robotics_repo_path)):
# Lese Umgebungsvariable für Codegenerierung.
read "../robot_codegen_definitions/robot_env":
printf("Generiere Parameter für %s\n",robot_name):
# Lese allgemeine Definitionen für Floating-Base
read sprintf("../codeexport/%s/tmp/tree_floatb_twist_definitions", robot_name):
# Robotics Definitions
# Position und Orientierung der Basis (pelvis). Die Orientierung ist mit XYZ-Euler-Winkeln definiert.
# Eine Invertierung der Orientierungsdarstellung sollte nicht notwendig werden, von daher kein Problem mit Orientierungsrepräsentationssingularität.
# gem. [2], S. 4 X_base_t SE(3): Position und Orientierung.
NQB := 6:
X_base_t:=Matrix(6, 1, [rx_base(t),ry_base(t),rz_base(t),alphax_base(t), betay_base(t), gammaz_base(t)]):
X_base_s:=Matrix(6, 1, [rxs_base,rys_base,rzs_base,alphaxs_base, betays_base, gammazs_base]):
# Geschwindigkeit der Basis
# Twist: Verallgemeinerte Geschwindigkeit
V_base_t:=diff~(X_base_t, t):
V_base_s:=Matrix(6, 1, [vxs_base,vys_base,vzs_base,alphaDx_base, betaDy_base, gammaDz_base]):
# Zeitableitung davon
VD_base_t:=diff~(V_base_t, t):
VD_base_s:=Matrix(6, 1, [vDxs_base,vDys_base,vDzs_base,alphaDDx_base, betaDDy_base, gammaDDz_base]):
# Name der Methode für die Orientierungsdarstellung der Basis
base_method_name := "eulxyz":
# Umrechnung von der Ableitung der Basis-Orientierung zu Winkelgeschwindigkeiten:
# Nutze die Euler-XYZ-Konvention ("RPY").
# Siehe [5], Gl. (4.23)
T_basevel := eulxyzjac(X_base_t(4..6,1)):
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
# Export
save q_t, q_s, qD_t, qD_s, qDD_t, qDD_s, qJ_t, qJ_s, qJD_t, qJD_s, qJDD_t, qJDD_s, g_world, X_base_t, X_base_s, V_base_t, V_base_s, VD_base_t, VD_base_s, qoffset, theta, alpha, d, a,v,b,beta, sigma,mu, M, r_i_i_Si, mr_i_i_Si, I_i_i, I_i_Si, PV2_vec, PV2_mat, robot_name, NQ,NQB,NQJ,NJ,NL, base_method_name, T_basevel, kintmp_t, kintmp_s, sprintf("../codeexport/%s/tmp/tree_floatb_eulxyz_definitions", robot_name):
save q_t, q_s, qD_t, qD_s, qDD_t, qDD_s, qJ_t, qJ_s, qJD_t, qJD_s, qJDD_t, qJDD_s, g_world, X_base_t, X_base_s, V_base_t, V_base_s, VD_base_t, VD_base_s, qoffset, theta, alpha, d, a,v,b,beta, sigma,mu, M, r_i_i_Si, mr_i_i_Si, I_i_i, I_i_Si, PV2_vec, PV2_mat, robot_name, NQ,NQB,NQJ,NJ,NL, base_method_name, T_basevel, kintmp_t, kintmp_s, sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name):

