
# Berechnung und Projektion der Dynamikgleichungen in Regressorform - Export
# Einleitung
# Berechnung und Projektion der Dynamikgleichungen in Regressorform
# Hier wird nur der Matlab-Export durchgeführt, getrennt von der eigentlichen Berechnung im gleichnamigen Arbeitsblatt ohne "_export"
# Autor
# Moritz Schappler, moritz.schappler@imes.uni-hannover.de
# (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover
# Initialization
interface(warnlevel=0): # Unterdrücke die folgende Warnung.
restart: # Gibt eine Warnung, wenn über Terminal-Maple mit read gestartet wird.
interface(warnlevel=3):
interface(rtablesize=100):
with(LinearAlgebra):
with(codegen):
with(CodeGeneration):
with(StringTools): # Für Zeitausgabe
;
# Einstellungen für Code-Export: Optimierungsgrad (2=höchster) und Aktivierung jedes Terms.
#codegen_act := true: # noch nicht implementiert
codegen_opt := 2:
codeexport_actcoord := false: # Generierung der Dynamik in Antriebskoordinaten nicht standardmäßig (hoher Rechenaufwand)
codeexport_grav := true: 
codeexport_corvec := true:
codeexport_inertia := true:
codeexport_invdyn := true:
read "../helper/proc_MatlabExport": 
# Roboter-Definitionen laden
read "../robot_codegen_definitions/robot_env_par":
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", leg_name):
read "../robot_codegen_definitions/robot_env_par":
# Es ist in diesem Arbeitsblatt möglich, zwei verschiedene Regressoren zu generieren und zu exportieren: Basierend auf Minimalparametern und auf vollem Parametersatz (PV2).
# Der Term "regressor" oder "regressor_minpar" ist jeweils in den Dateinamen enthalten.
# Der folgende Befehl muss immer auf "regressor_minpar" gesetzt sein, da diese Zeile durch das Skript robot_codegen_maple_preparation.sh ausgewertet und modifiziert wird.
regressor_modus := "regressor_minpar": 
# Ausgabe der zu exportierenden Terme
DynString := "Term:":
if codeexport_grav then
  DynString := sprintf("%s g",DynString):
end if:
if codeexport_corvec then
  DynString := sprintf("%s c",DynString):
end if:
if codeexport_inertia then
  DynString := sprintf("%s M",DynString):
end if:
if codeexport_invdyn then
  DynString := sprintf("%s tau",DynString):
end if:
printf("%s. Generiere Dynamik-Regressorform (%s) für %s mit %s\n", FormatTime("%Y-%m-%d %H:%M:%S"), DynString, robot_name, regressor_modus):

# Kinematik laden (zur Einschätzung der Komplexität)
# Ergebnisse der Kinematik für parallelen Roboter laden
read "../robot_codegen_definitions/robot_env_par":
kinematicsfile := sprintf("../codeexport/%s/tmp/kinematics_%s_platform_maple.m", robot_name, base_method_name):
if FileTools[Exists](kinematicsfile) then
  read kinematicsfile:
else
  printf("%s. PKM-Kinematik konnte nicht geladen werden. Abbruch der Berechnung.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  quit: # Funktioniert in GUI nicht richtig...
  robot_name := "": # ...Daher auch Löschung des Roboternamens.
end if:
read "../robot_codegen_definitions/robot_env_par": # Nochmal laden, um Standard-Einstellungen überschreiben zu können.
# Neu-Definition der von dieser Datei gelesenen Variablen, damit sie im Workspace erscheinen
Jinv := Jinv:
# Import der Maple-Ausdrücke
# Werden zuvor in gleichnamigem Arbeitsblatt gespeichert
if codeexport_invdyn then
  read sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_%s_maple.m",      robot_name, regressor_modus):
  tau_x := tau_x:
  if regressor_modus = "regressor_minpar" then
    read sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_reg_mdp_maple.m",     robot_name):
    tau_x_mdp := tau_x_mdp:
  end if:
end if:
if codeexport_inertia then
  read sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_MM_%s_maple.m",   robot_name, regressor_modus):
  MMreg_x := MMreg_x:
  if regressor_modus = "regressor_minpar" then
    read sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_MMreg_mdp_maple.m",   robot_name):
    MMreg_x_mdp := MMreg_x_mdp:
  end if:
end if:
if codeexport_corvec then
  read sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_tauC_%s_maple.m", robot_name, regressor_modus):
  Creg_x := Creg_x:
  if regressor_modus = "regressor_minpar" then
    read sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_tauCreg_mdp_maple.m", robot_name):
    Creg_x_mdp := Creg_x_mdp:
  end if:
end if:
if codeexport_grav then
  read sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_taug_%s_maple.m", robot_name, regressor_modus):
  greg_x := greg_x:
  if regressor_modus = "regressor_minpar" then
    read sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_taugreg_mdp_maple.m", robot_name):
    greg_x_mdp := greg_x_mdp:
  end if:
end if: 

# Export der Matlab-Ausdrücke
# In Plattform-Koordinaten
if codeexport_invdyn then
  printf("%s. Beginne Code-Export Inverse Dynamik (Regressor) in Plattform-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  MatlabExport(tau_x, sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_%s_matlab.m", robot_name, regressor_modus), codegen_opt):
end if:
if codeexport_inertia then
  printf("%s. Beginne Code-Export Massenmatrix (Regressor) in Plattform-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  MatlabExport(MMreg_x, sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_MM_%s_matlab.m", robot_name, regressor_modus), codegen_opt):
end if:
if codeexport_corvec then
  printf("%s. Beginne Code-Export Coriolis-Vektor (Regressor) in Plattform-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  MatlabExport(Creg_x, sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_tauC_%s_matlab.m", robot_name, regressor_modus), codegen_opt):
end if:
if codeexport_grav then
  printf("%s. Beginne Code-Export Gravitations-Vektor (Regressor) in Plattform-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  MatlabExport(greg_x, sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_taug_%s_matlab.m", robot_name, regressor_modus), codegen_opt):
end if:
# In Antriebskoordinaten
if RowDimension(Jinv) < 5 and codeexport_actcoord then
  if codeexport_invdyn then
    printf("%s. Beginne Code-Export Inverse Dynamik (Regressor) in Antriebs-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
    MatlabExport(tau_qa, sprintf("../codeexport/%s/tmp/invdyn_para_actcoord_%s_matlab.m", robot_name, regressor_modus), codegen_opt):
  end if:
  if codeexport_inertia then
    printf("%s. Beginne Code-Export Massenmatrix (Regressor) in Antriebs-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
    MatlabExport(MMreg_qa, sprintf("../codeexport/%s/tmp/invdyn_para_actcoord_MM_%s_matlab.m", robot_name, regressor_modus), codegen_opt):
  end if:
  if codeexport_corvec then
    printf("%s. Beginne Code-Export Coriolis-Vektor (Regressor) in Antriebs-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
    MatlabExport(Creg_qa, sprintf("../codeexport/%s/tmp/invdyn_para_actcoord_tauC_%s_matlab.m", robot_name, regressor_modus), codegen_opt):
  end if:
  if codeexport_grav then
    printf("%s. Beginne Code-Export Gravitations-Vektor (Regressor) in Antriebs-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
    MatlabExport(greg_qa, sprintf("../codeexport/%s/tmp/invdyn_para_actcoord_taug_%s_matlab.m", robot_name, regressor_modus), codegen_opt):
  end if:
end if:
# Plattform-Koordinaten, MPV eingesetzt
if regressor_modus = "regressor_minpar" then
  if codeexport_invdyn then
    printf("%s. Beginne Code-Export Inverse Dynamik (MDP eingesetzt) in Plattform-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
    MatlabExport(tau_x_mdp, sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_reg_mdp_matlab.m", robot_name), codegen_opt):
  end if:
  if codeexport_inertia then
    printf("%s. Beginne Code-Export Massenmatrix (MDP eingesetzt) in Plattform-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
    MatlabExport(MMreg_x_mdp, sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_MMreg_mdp_matlab.m", robot_name), codegen_opt):
  end if:
  if codeexport_corvec then
    printf("%s. Beginne Code-Export Coriolis-Vektor (MDP eingesetzt) in Plattform-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
    MatlabExport(Creg_x_mdp, sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_tauCreg_mdp_matlab.m", robot_name), codegen_opt):
  end if:
  if codeexport_grav then
    printf("%s. Beginne Code-Export Gravitations-Vektor (MDP eingesetzt) in Plattform-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
    MatlabExport(greg_x_mdp, sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_taugreg_mdp_matlab.m", robot_name), codegen_opt):
  end if:
end if:
# Antriebskoordinaten, MPV eingesetzt
if RowDimension(Jinv) < 5 and codeexport_actcoord and regressor_modus = "regressor_minpar" then
  if codeexport_invdyn then
    printf("%s. Beginne Code-Export Inverse Dynamik (MDP eingesetzt) in Antriebs-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
    MatlabExport(tau_qa_mdp, sprintf("../codeexport/%s/tmp/invdyn_para_actcoord_reg_mdp_matlab.m", robot_name), codegen_opt):
  end if:
  if codeexport_inertia then
    printf("%s. Beginne Code-Export Massenmatrix (MDP eingesetzt) in Antriebs-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
    MatlabExport(MMreg_qa_mdp, sprintf("../codeexport/%s/tmp/invdyn_para_actcoord_MMreg_mdp_matlab.m", robot_name), codegen_opt):
  end if:
  if codeexport_corvec then
    printf("%s. Beginne Code-Export Coriolis-Vektor (MDP eingesetzt) in Antriebs-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
    MatlabExport(Creg_qa_mdp, sprintf("../codeexport/%s/tmp/invdyn_para_actcoord_tauCreg_mdp_matlab.m", robot_name), codegen_opt):
  end if:
  if codeexport_grav then
    printf("%s. Beginne Code-Export Gravitations-Vektor (MDP eingesetzt) in Antriebs-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
    MatlabExport(greg_qa_mdp, sprintf("../codeexport/%s/tmp/invdyn_para_actcoord_taugreg_mdp_matlab.m", robot_name), codegen_opt):
  end if:
end if:


