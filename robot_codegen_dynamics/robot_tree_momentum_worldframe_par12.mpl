
# Momentum Calculation for General Robots
# Introduction
# Berechnung des Impulses und Drehimpulses und davon abgeleiteter Größen wie Zeitableitungen, CMM und ZMP.
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# tree -> Berechnung für eine beliebige Baumstruktur (ohne Schleifen)
# momentum -> Berechnung des Impulses (translatorisch und rotatorisch)
# worldframe -> Berechnung der Geschwindigkeit im Basis-KS (KS0)
# par12 -> Parametersatz 1 (Schwerpunkt als Parameter: SX,SY,SZ) oder Parametersatz 2 (1. und 2. Moment MX,MY,MZ,...)
# Authors
# Moritz Schappler, schappler@irt.uni-hannover.de, 2014-11
# Jonathan Vorndamme, vorndamme@irt.uni-hannover.de, 2015-12
# Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# References
# [LeeGos2011] Lee, S.-H. & Goswami, A.Fall on Backpack: Damage Minimizing Humanoid Fall on Targeted Body Segment Using Momentum Control (2011)
# [OrinGos2008] Orin, D.E. and Goswami, A.: Centroidal Momentum Matrix of a humanoid robot: Structure and properties (2008)
# [KajitaEsp2008] Kajita, Shuuji and Espiau, Bernard: Legged Robots (Springer Handbook of Robotics) 2008)
# Initialization
interface(warnlevel=0): # Unterdrücke die folgende Warnung.
restart: # Gibt eine Warnung, wenn über Terminal-Maple mit read gestartet wird.
interface(warnlevel=3):
# Einstellungen für Code-Export: Optimierungsgrad (2=höchster) und Aktivierung jedes Terms.
codegen_act := true:
codegen_opt := 2:
with(LinearAlgebra):
with(ArrayTools):
with(codegen):
with(CodeGeneration):
with(StringTools):
read "../helper/proc_convert_s_t":
read "../helper/proc_convert_t_s": 
read "../helper/proc_MatlabExport":
read "../robot_codegen_definitions/robot_env":
# Prüfe, ob die Berechnung notwendig ist

abort_this_worksheet := true:

if assigned(compute_momenta) and compute_momenta then
  abort_this_worksheet := false:
end if:

if abort_this_worksheet then
  printf("Analytische Berechnung der Impuls-basierten Funktionen nicht aktiviert.\n"):
  quit: # Funktioniert in GUI nicht richtig...
  robot_name := "": # ...Daher auch Löschung des Roboternamens.
  codegen_act := false:
end if:
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name):
printf("Generiere Impuls und Drehimpuls für %s mit Basis-Darstellung %s\n", robot_name, base_method_name):
# Kennung des Parametersatzes, für den die Dynamikfunktionen erstellt werden sollen. Muss im Repo und in der mpl-Datei auf 1 gelassen werden, da die folgende Zeile mit einem Skript verarbeitet wird.
codegen_dynpar := 2:
# Ergebnisse der Kinematik laden
read sprintf("../codeexport/%s/tmp/kinematics_floatb_%s_rotmat_maple.m", robot_name, base_method_name):
Trf := Trf:
Trf_c := Trf_c:
# Get Position of Center of Mass
# Schwerpunkte aller Teilkörper
read sprintf("../codeexport/%s/tmp/kinematics_com_worldframe_floatb_%s_par1_maple.m", robot_name, base_method_name):
mr_W_i_Si := mr_W_i_Si:
r_W_W_Si := r_W_W_Si:
r_W_i_Si := r_W_i_Si:
# Schwerpunkt des Gesamtsystems
read sprintf("../codeexport/%s/tmp/kinematics_com_total_worldframe_floatb_%s_par%d_maple.m", robot_name, base_method_name, codegen_dynpar):
c:=c:
# [KajitaEsp2008] S. 375
c_inert:=Matrix(3,1,<c_x; c_y; c_z>): # Platzhalter, nicht ausgewertet
;
# Get Calculated Velocities
# Lese Variablen omega_W_i, rD_W_i, rD_W_Si (sind unabhängig von par1. Name in anderem Skript ist ungenau)
# TODO: Schwerpunktsgeschwindigkeiten mit anderem Parametersatz
read sprintf("../codeexport/%s/tmp/velocity_worldframe_floatbase_%s_par%d_maple.m", robot_name, base_method_name, 1):
omega_W_i := omega_W_i:
rD_W_i := rD_W_i:
rD_W_Si := rD_W_Si:
read sprintf("../codeexport/%s/tmp/velocity_linkframe_floatb_%s_maple.m", robot_name, base_method_name):
omega_i_i := omega_i_i: 
rD_i_i := rD_i_i:
# Calculate linear momentum and time differential of linear momentum
# [KajitaEsp2008] Gl. (16.29)

P_t:=Matrix(3,1):
# Für Parametersatz 1 direkt im Welt-KS berechnen
if codegen_dynpar = 1 then
  for i from 1 to NL do 
    P_t:=P_t + Matrix(rD_W_Si(1 .. 3,i))*M[i,1]:
  end do
end if:
# Für Parametersatz 2: Bestimme den Impuls analog zur kinetischen Energie mit lokalen Koordinatensystemen
if codegen_dynpar = 2 then
  for i from 1 to NL do
    P_i:= Matrix(rD_i_i(1 .. 3,i))*M[i,1] + Matrix(CrossProduct(omega_i_i(1 .. 3, i), mr_i_i_Si(1 .. 3, i))):
    R_i:=Trf_c(1 .. 3, 1 .. 3, i):
    P_t := P_t + Multiply(R_i, P_i):
  end do
end if:
if codegen_act then
  MatlabExport(convert_t_s(P_t), sprintf("../codeexport/%s/tmp/linmomentum_worldframe_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt)
end if:
PD_t:=diff~(P_t,t):
if codegen_act then
  MatlabExport(convert_t_s(PD_t), sprintf("../codeexport/%s/tmp/linmomentumD_worldframe_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
# [KajitaEsp2008] S. 375
PD_inert:=Matrix(3,1,<PD_x; PD_y; PD_z>): # Platzhalter, nicht ausgewertet
;


# Calculate angular momentum and time differential of angular momentum
# [KajitaEsp2008] Gl. 16.30
L_t:=Matrix(3,1):
# Gleiche Vorgehensweise für Drehimpuls wie beim Impuls
if codegen_dynpar = 1 then
  for i from 1 to NL do
    R_i:=Trf_c(1 .. 3, 1 .. 3, i):
    I_i_Si_Tensor := Matrix([[I_i_Si[1, i], I_i_Si[2, i], I_i_Si[3, i]], [I_i_Si[2, i], I_i_Si[4, i], I_i_Si[5, i]], [I_i_Si[3, i],   I_i_Si[5, i], I_i_Si[6, i]]]):
    L_t:=L_t+Matrix(CrossProduct(r_W_W_Si(1 .. 3, i), M[i,1]*rD_W_Si(1 .. 3, i))):
    L_t:=L_t+R_i.I_i_Si_Tensor.Transpose(R_i).Matrix(omega_W_i(1..3,i)):
  end do:
end if:
# 
# TODO: Funktioniert noch nicht mit Parametersatz 2. Ist der Impuls eventuell gar nicht damit bestimmbar?
# Es muss der Steiner-Anteil wieder rausgerechnet werden, damit der gleiche Ausdruck wie bei Parametersatz 1 rauskommt.
# Andere Frage: Ist es da überhaupt richtig? Das müssten die Tests beantworten.
if codegen_dynpar = 2 then
  for i from 1 to NL do
    R_i:=Trf_c(1 .. 3, 1 .. 3, i):
    I_i_i_tensor := Matrix([[I_i_i[1, i], I_i_i[2, i], I_i_i[3, i]], [I_i_i[2, i], I_i_i[4, i], I_i_i[5, i]], [I_i_i[3, i], I_i_i[5, i], I_i_i[6, i]]]):
   # mrD_i_Si := rD_i_i(1 .. 3, i) + CrossProduct(mr_i_i_Si(1 .. 3, i), (omega_i_i, ))
    # falsch: L_i := -2*Matrix(CrossProduct(Matrix(omega_i_i(1 .. 3, i)), Matrix(mr_i_i_Si(1 .. 3, i)))):
    # L_i := Matrix(CrossProduct(Matrix(rD_i_i(1 .. 3, i)), Matrix(mr_i_i_Si(1 .. 3, i)))):
    L_i := Matrix(3,1):
    L_i := L_i + Multiply(I_i_i_tensor, Matrix(omega_i_i(1 .. 3, i))):
    L_t := L_t + Multiply(R_i, Matrix(L_i)):
  end do:
end if:
if codegen_act then
  MatlabExport(convert_t_s(L_t), sprintf("../codeexport/%s/tmp/angmomentum_worldframe_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
LD_t:=diff~(L_t,t):
if codegen_act then
  MatlabExport(convert_t_s(LD_t), sprintf("../codeexport/%s/tmp/angmomentumD_worldframe_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
# [KajitaEsp2008] S. 375
LD_inert:=Matrix(3,1,<LD_x; LD_y; LD_z>): # Platzhalter, nicht ausgewertet
;


# Centroidal Momentum Matrix
# Ableitung des verallg. Impulses nach den Basis- und Gelenkwinkelgeschwindigkeiten nach [OrinGos2008]
# [OrinGos2008] Gl. , (1)  enthält nur Gelenkwinkel?
# [LeeGos2011] Gl. (1) enthält auch die Basisgeschwindigkeit
PL_s := convert_t_s(<P_t;L_t>):
K_s := Matrix(6,NQ): # K in [LeeGos2011] mit anderer Reihenfolge
;
for i from 1 to NQ do
  K_s(1..6,i) := diff~(PL_s, qD_s(i,1)):
end do:
if codegen_act then
  MatlabExport(K_s, sprintf("../codeexport/%s/tmp/centroidal_momentum_matrix_worldframe_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
# Zeitableitung der Centroidal Momentum Matrix
K_t := convert_s_t(K_s):
KD_t := diff~(K_t, t): # \dot{K} in [LeeGos2011] mit anderer Reihenfolge
;
KD_s := convert_t_s(KD_t):
if codegen_act then
  MatlabExport(KD_s, sprintf("../codeexport/%s/tmp/centroidal_momentum_matrix_diff_worldframe_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
# Calculate ZMP x and y
# Berechnung ohne Einsetzen der einzelnen Ausdrücke
# Für P,L,C werden einfach Platzhalter eingesetzt. Keine größere symbolische Berechnung
# Berechne den ZMP im Welt-KS.
# Quelle: [KajitaEsp2008] 
p:=Matrix(3,1,<p_x;p_y;p_z>):
# [KajitaEsp2008] Gl. (16.34)
tau_ZMP:=LD_inert - Matrix(CrossProduct(c_inert,M_ges*g_world)) + Matrix(CrossProduct((PD_inert-M_ges*g_world),p)):
ZMP_y:=solve([tau_ZMP(1)=0], [p_y]):
ZMP_x:=solve([tau_ZMP(2)=0], [p_x]):

# [KajitaEsp2008] Gl. (16.37), (16.38)
ZMP_x:=rhs(ZMP_x[1][1]):
ZMP_y:=rhs(ZMP_y[1][1]):
ZMP_x:=simplify(ZMP_x):
ZMP_y:=simplify(ZMP_y):
if codegen_act then
  MatlabExport(convert_t_s(ZMP_x), sprintf("../codeexport/%s/tmp/ZMP_x_worldframe_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
  MatlabExport(convert_t_s(ZMP_y), sprintf("../codeexport/%s/tmp/ZMP_y_worldframe_floatb_%s_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:

