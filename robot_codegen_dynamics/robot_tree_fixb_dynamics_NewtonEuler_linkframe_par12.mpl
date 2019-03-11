
# NewtonEuler Formulation for Robot based on MDH frames

# Einleitung
# Berechnung der inversen Dynamik in Rückwärtsrekursive
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# tree -> Berechnung für eine beliebige Baumstruktur (ohne Schleifen)
# fixb ->  Fixed Base Modell der Basis.
# dynamics -> Berechnung der Dynamik
# NewtonEuler->Newton - Euler Bewegungsgleichung
# Linkframe->Berechnung der Newton Euler im Körper-KS(KSi)
# par12 -> Parametersatz 1 (Schwerpunkt als Parameter: SX,SY,SZ) oder Parametersatz 2 (1. und 2. Moment MX,MY,MZ,...)

# Sources
# [GautierKhalil1990] Direct Calculation of Minimum Set of Inertial Parameters of Serial Robots
# [KhalilDombre2002] Modeling, Identification and Control of Robots
# [Ortmaier2014] Vorlesungsskript Robotik I
# 
# Initialization
interface(warnlevel=0): # Unterdrücke die folgende Warnung.
restart: # Gibt eine Warnung, wenn über Terminal-Maple mit read gestartet wird.
interface(warnlevel=3):
with(LinearAlgebra):
with(ArrayTools):
with(codegen):
with(CodeGeneration):
with(StringTools):
read "../transformation/proc_rotx": 
read "../transformation/proc_roty": 
read "../transformation/proc_rotz": 
read "../transformation/proc_trotx": 
read "../transformation/proc_troty": 
read "../transformation/proc_trotz": 
read "../transformation/proc_transl": 
read "../transformation/proc_trafo_mdh": 
# Einstellungen für Code-Export: Optimierungsgrad (2=höchster) und Aktivierung jedes Terms.
codegen_opt := 2:
codeexport_grav := true: 
codeexport_corvec := true:
codeexport_cormat := true:
codeexport_inertia := true:
codeexport_inertiaD := true:
codeexport_invdyn := true:
codegen_act := true:
codegen_dynpar := 2:
read "../helper/proc_convert_s_t":
read "../helper/proc_convert_t_s": 
read "../helper/proc_MatlabExport":
read "../robot_codegen_definitions/robot_env":
printf("Generiere Geschwindigkeit für %s (Herleitung im Körper-KS)\n", robot_name):
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name):
read "../helper/proc_index_symmat2vector":
read "../helper/proc_symmat2vector":
# Ergebnisse der Kinematik laden
read sprintf("../codeexport/%s/tmp/kinematics_floatb_%s_rotmat_maple.m", robot_name, base_method_name):
Trf := Trf:
Trf_c := Trf_c:
# Ergebnisse der Geschwindigkeit laden
read sprintf("../codeexport/%s/tmp/velocity_linkframe_floatb_%s_maple.m", robot_name, base_method_name):
omega_i_i:= omega_i_i:
rD_i_i:= rD_i_i:
# Zeitableitungen der MDH-Drehwinkel laden.
# Die Berechnung soll nur an einer Stelle erfolgen. Siehe robot_tree_velocity_mdh_angles.mw.
read sprintf("../codeexport/%s/tmp/velocity_mdh_angles_maple.m", robot_name):
thetaD := thetaD:
dD :=dD:
# Zeitableitungen der Geschwindigkeit laden.
# Die Berechnung soll nur an einer Stelle erfolgen. Siehe robot_tree_acceleration_mdh_angles.mw.
read sprintf("../codeexport/%s/tmp/acceleration_mdh_angles_maple.m", robot_name):
thetaDD :=thetaDD:
dDD := dDD:
# Ergebnisse der Geschwindigkeit laden
read sprintf("../codeexport/%s/tmp/velocity_linkframe_floatb_%s_maple.m", robot_name, base_method_name):
omega_i_i:= omega_i_i:
rD_i_i:= rD_i_i:
# Ergebnisse der Beschleunigung laden
 read sprintf("../codeexport/%s/tmp/acceleration_linkframe_floatb_%s_maple.m", robot_name, base_method_name):
omegaD_i_i:= omegaD_i_i: 
rDD_i_i:= rDD_i_i:
rDD_i_Si:= rDD_i_Si:
# 
# Mit diesem Arbeitsblatt werden die Vorwärtsrekursive für Fixed-Base Modelle generiert. Erkenne welche Basis-Modellierung aktiv ist
if base_method_name="twist" then # Basis-Methode "twist" wird (hier) nur für fixed Base benutzt
  expstring:="fixb":
elif base_method_name="eulxyz" then 
  expstring:="floatb_eulxyz":
else
  printf("Nicht behandelte Basis-Methode: %s\n", base_method_name):
end if:
# Schalter zur Auswahl der unterschiedlichen Terme, die exportiert werden sollen.
# Newton-Euler formulation(mit Funktion)
OutputNewtonEuler:= NewtonEuler(f_i_i,m_i_i):
# Backward recursive computation
# Force exerted on link i by link i-1(unbekannte Kraft zwischen Teilkörper n-1 und Teilkörper n)
f_i_i:= Matrix(3,NL):
# Moment exerted on link i by link i-1(unbekannte Moment zwischen Teilkörper n-1 und Teilkörper n)
m_i_i:=Matrix(3,NL):

# Schnittmomente/Kräfte
tau_B := Matrix(6,1):
tau_J := Matrix(NJ,1):
tau := Matrix(6+NJ,1):


I_i_i_m:= Matrix(3,3,NL):
for i from 1 to NL do
  I_i_i_m(1..3,1..3,i):=Matrix(3,3):
 end do:

for i from 1 to NL do
  I_i_i_m[1,1,i]:=I_i_i[1,i]:
  I_i_i_m[1,2,i]:=I_i_i[2,i]:
  I_i_i_m[1,3,i]:=I_i_i[3,i]:
  I_i_i_m[2,2,i]:=I_i_i[4,i]:
  I_i_i_m[2,3,i]:=I_i_i[5,i]:
  I_i_i_m[3,3,i]:=I_i_i[6,i]:
end do:


F_i:= Matrix(3,NL):
M_i:= Matrix(3,NL):
for i from 1 to NL do
  # External forces and moment on link i
  #Betrachte dazu nur die durch Gelenke angetriebenen Körper, nicht die Basis
  # [Khali2002](9.5.3) (9.88) & (9.89)
  F_i(1..3,i):= M(i,1)*rDD_i_i(1..3,i)+CrossProduct(omegaD_i_i(1..3,i),mr_i_i_Si(1..3,i))+ CrossProduct(omega_i_i(1..3,i), CrossProduct(omega_i_i(1..3,i), mr_i_i_Si(1..3,i))):
  M_i(1..3,i):= I_i_i_m(1..3,1..3,i).omegaD_i_i(1..3,i)+ CrossProduct(omega_i_i(1..3,i),(I_i_i_m(1..3,1..3,i).omega_i_i(1..3,i)))+CrossProduct(mr_i_i_Si(1..3,i),rDD_i_i(1..3,i)):
end do :



# Forces and moment exerted on link i by link i-1

for i from NL by -1 to 1 do
  f_i_i(1..3,i):= F_i(1..3,i):
  m_i_i(1..3,i):= M_i(1..3,i):#+ CrossProduct(r_i_i_Si(1..3,i),(F_i(1..3,i))):

  if i < NL then
    j := v(i) + 2:
    # printf("i=%d, j=%d\n", i, j):
    R_j_i := Matrix(Trf(1..3,1..3,i)): 
    r_j_j_i := Matrix( Trf(1 .. 3, 4, i)):
    f_i_i(1..3,i) := f_i_i(1..3,i) + R_j_i.f_i_i(1..3,j):
    m_i_i(1..3,i) := m_i_i(1..3,i) + R_j_i.m_i_i(1..3,j)+ CrossProduct(r_j_j_i, R_j_i.f_i_i(1..3,j) ):
  end if:
end do:

for i from 1 to NJ do
  if sigma(i) = 0 then # Drehgelenk
    tau_J(i,1) := <0,0,1>.m_i_i(1..3,i+1):
  else # Schubgelenk
    tau_J(i,1) := <0,0,1>.f_i_i(1..3,i+1):
  end if:
end do:
# Unbekannten Kräfte und Momente zwischen den Teilkörpern bestimmen
# 
tau_B [1..3,1]:= Transpose(Trf_c(1..3, 1..3, 1)).f_i_i(1..3,1) :
tau_B [4..6,1]:= Transpose(T_basevel).Trf_c(1..3, 1..3, 1).m_i_i(1..3,1):
tau[1..6,1]:= tau_B:
tau[7..6+NJ,1]:= tau_J:
# Export
f_i_i := convert_t_s(f_i_i):
m_i_i := convert_t_s(m_i_i):
tau_J := convert_t_s(tau_J):
tau_B := convert_t_s(tau_B):
tau := convert_t_s(tau):
# Floating Base
# Maple Export
if codeexport_invdyn and not(base_method_name="twist") then
  save  f_i_i,m_i_i,tau_J, tau_B, tau,sprintf("../codeexport/%s/tmp/invdyn_%s_NewtonEuler_linkframe_maple.m", robot_name, base_method_name):
  printf("Maple-Ausdrücke exportiert. %s\n", FormatTime("%Y-%m-%d %H:%M:%S")):
end if:
# Matlab Export
if codeexport_invdyn and not(base_method_name="twist") then
  MatlabExport(convert_t_s(f_i_i), sprintf("../codeexport/%s/tmp/invdyn_floatb_%s_NewtonEuler_linkframe_f_i_i_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
  MatlabExport(convert_t_s(m_i_i), sprintf("../codeexport/%s/tmp/invdyn_floatb_%s_NewtonEuler_linkframe_m_i_i_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
  MatlabExport(convert_t_s(tau_J), sprintf("../codeexport/%s/tmp/invdyn_floatb_%s_NewtonEuler_linkframe_tauJ_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
  MatlabExport(convert_t_s(tau_B), sprintf("../codeexport/%s/tmp/invdyn_floatb_%s_NewtonEuler_linkframe_tauB_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
  MatlabExport(convert_t_s(tau), sprintf("../codeexport/%s/tmp/invdyn_floatb_%s_NewtonEuler_linkframe_tauJB_par%d_matlab.m", robot_name, base_method_name, codegen_dynpar), codegen_opt):
end if:
# Fixed Base
for i from 1 to NQB do
  f_i_i := subs({X_base_s[i,1]=0},f_i_i):
  m_i_i := subs({X_base_s[i,1]=0},m_i_i):
  tau_J := subs({X_base_s[i,1]=0},tau_J):
  tau_B := subs({X_base_s[i,1]=0},tau_B):
  tau := subs({X_base_s[i,1]=0},tau):
end do:
for i from 1 to 6 do
  f_i_i := subs({V_base_s[i,1]=0},f_i_i):
  m_i_i := subs({V_base_s[i,1]=0},m_i_i):
  tau_J := subs({V_base_s[i,1]=0},tau_J):
  tau_B := subs({V_base_s[i,1]=0},tau_B):
  tau := subs({V_base_s[i,1]=0},tau):
  f_i_i := subs({VD_base_s[i,1]=0},f_i_i):
  m_i_i := subs({VD_base_s[i,1]=0},m_i_i):
  tau_J := subs({VD_base_s[i,1]=0},tau_J):
  tau_B := subs({VD_base_s[i,1]=0},tau_B):
  tau := subs({VD_base_s[i,1]=0},tau):
end do:
# Maple Export
if codeexport_invdyn then
  save  f_i_i,m_i_i,tau_J, tau_B, tau,sprintf("../codeexport/%s/tmp/invdyn_%s_NewtonEuler_linkframe_maple.m", robot_name, base_method_name):
  printf("Maple-Ausdrücke exportiert. %s\n", FormatTime("%Y-%m-%d %H:%M:%S")):
end if:
# Matlab Export
if codeexport_invdyn  then
  MatlabExport(f_i_i, sprintf("../codeexport/%s/tmp/invdyn_fixb_NewtonEuler_linkframe_f_i_i_par%d_matlab.m", robot_name, codegen_dynpar), codegen_opt):
  MatlabExport(m_i_i, sprintf("../codeexport/%s/tmp/invdyn_fixb_NewtonEuler_linkframe_m_i_i_par%d_matlab.m", robot_name, codegen_dynpar), codegen_opt):
  MatlabExport(tau_J, sprintf("../codeexport/%s/tmp/invdyn_fixb_NewtonEuler_linkframe_tauJ_par%d_matlab.m", robot_name, codegen_dynpar), codegen_opt):
  MatlabExport(tau_B, sprintf("../codeexport/%s/tmp/invdyn_fixb_NewtonEuler_linkframe_tauB_par%d_matlab.m", robot_name, codegen_dynpar), codegen_opt):
  MatlabExport(tau, sprintf("../codeexport/%s/tmp/invdyn_fixb_NewtonEuler_linkframe_tauJB_par%d_matlab.m", robot_name, codegen_dynpar), codegen_opt):
end if:

# 
