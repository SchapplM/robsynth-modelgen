
# Berechnung und Projektion der Dynamikgleichungen - Export
# Einleitung
# Berechnung und Projektion der Dynamikgleichungen.
# Hier wird nur der Matlab-Export durchgeführt, getrennt von der eigentlichen Berechnung im gleichnamigen Arbeitsblatt ohne "_export"
# Autor
# Moritz Schappler, moritz.schappler@imes.uni-hannover.de
# (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover
# Initialization
interface(warnlevel=0): # Unterdrücke die folgende Warnung.
restart: # Gibt eine Warnung, wenn über Terminal-Maple mit read gestartet wird.
interface(warnlevel=3):
with(LinearAlgebra):
with(codegen):
with(CodeGeneration):
with(StringTools): # Für Zeitausgabe
;
# Einstellungen für Code-Export: Optimierungsgrad (2=höchster).
#codegen_act := true: # noch nicht implementiert
codegen_debug := false:
codegen_opt := 2:
codeexport_actcoord := false: # Generierung der Dynamik in Antriebskoordinaten nicht standardmäßig (hoher Rechenaufwand)
codeexport_grav := true: 
codeexport_corvec := true:
codeexport_inertia := true:
codeexport_invdyn := true:
read "../helper/proc_MatlabExport":
read "../robot_codegen_definitions/robot_env_par":
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", leg_name):
# Kennung des Parametersatzes, für den die Dynamikfunktionen erstellt werden sollen. Muss im Repo und in der mpl-Datei auf 1 gelassen werden, da die folgende Zeile mit einem Skript verarbeitet wird.
codegen_dynpar := 1:
# Ergebnisse der zusätzlichen Definitionen für parallele Roboter laden
read "../robot_codegen_definitions/robot_env_par":
read sprintf("../codeexport/%s/tmp/para_definitions", robot_name):
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
printf("%s. Generiere Dynamik-Terme (%s) für %s mit dynpar %d\n", FormatTime("%Y-%m-%d %H:%M:%S"), DynString, robot_name, codegen_dynpar):

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
if codeexport_grav then
  read sprintf("../codeexport/%s/tmp/gravvec_para_plfcoord_par%d_maple.m", robot_name, codegen_dynpar):
  gGes_x := gGes_x:
  if RowDimension(Jinv) < 5 and codeexport_actcoord then
    read sprintf("../codeexport/%s/tmp/gravvec_para_actcoord_par%d_maple.m", robot_name, codegen_dynpar):
    gGes_qa := gGes_qa:
  end if
end if:
if codeexport_corvec then
  read sprintf("../codeexport/%s/tmp/coriolisvec_para_plfcoord_par%d_maple.m", robot_name, codegen_dynpar):
  cvecGes_x := cvecGes_x:
  if RowDimension(Jinv) < 5 and codeexport_actcoord then
    read sprintf("../codeexport/%s/tmp/coriolisvec_para_actcoord_par%d_maple.m", robot_name, codegen_dynpar):
    cvecGes_qa := cvecGes_qa:
  end if
end if:
if codeexport_inertia then
  read sprintf("../codeexport/%s/tmp/inertia_para_plfcoord_par%d_maple.m", robot_name, codegen_dynpar):
  MMGes_x := MMGes_x:
  if RowDimension(Jinv) < 5 and codeexport_actcoord then
    read sprintf("../codeexport/%s/tmp/inertia_para_actcoord_par%d_maple.m", robot_name, codegen_dynpar):
    MMGes_qa := MMGes_qa:
  end if
end if:
if codeexport_invdyn then
  read sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_par%d_maple.m", robot_name, codegen_dynpar):
  tau_x := tau_x:
  if RowDimension(Jinv) < 5 and codeexport_actcoord then
    read sprintf("../codeexport/%s/tmp/invdyn_para_actcoord_par%d_maple.m", robot_name, codegen_dynpar):
    tau_qa := tau_qa:
  end if
end if:

# Matlab-Export
if codeexport_invdyn then
  printf("%s. Beginne Code-Export Inverse Dynamik in Plattform-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  MatlabExport(tau_x, sprintf("../codeexport/%s/tmp/invdyn_para_plfcoord_par%d_matlab.m", robot_name, codegen_dynpar), codegen_opt);
end if:
if codeexport_inertia then
  printf("%s. Beginne Code-Export Massenmatrix in Plattform-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  MatlabExport(MMGes_x, sprintf("../codeexport/%s/tmp/inertia_para_plfcoord_par%d_matlab.m", robot_name, codegen_dynpar), codegen_opt);
end if:
if codeexport_corvec then
  printf("%s. Beginne Code-Export Coriolis-Vektor in Plattform-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  MatlabExport(cvecGes_x, sprintf("../codeexport/%s/tmp/coriolisvec_para_plfcoord_par%d_matlab.m", robot_name, codegen_dynpar), codegen_opt);
end if:
if codeexport_grav then
  printf("%s. Beginne Code-Export Gravitations-Vektor in Plattform-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
  MatlabExport(gGes_x, sprintf("../codeexport/%s/tmp/gravvec_para_plfcoord_par%d_matlab.m", robot_name, codegen_dynpar), codegen_opt);
end if:
if RowDimension(Jinv) < 5 and codeexport_actcoord then
  if codeexport_invdyn then	
    printf("%s. Beginne Code-Export Inverse Dynamik in Antriebs-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
    MatlabExport(tau_qa, sprintf("../codeexport/%s/tmp/invdyn_para_actcoord_par%d_matlab.m", robot_name, codegen_dynpar), codegen_opt);
  end if:
  if codeexport_inertia then
    printf("%s. Beginne Code-Export Massenmatrix in Antriebs-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
    MatlabExport(MMGes_qa, sprintf("../codeexport/%s/tmp/inertia_para_actcoord_par%d_matlab.m", robot_name, codegen_dynpar), codegen_opt);
  end if:
  if codeexport_corvec then
    printf("%s. Beginne Code-Export Coriolis-Vektor in Antriebs-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
    MatlabExport(cvecGes_qa, sprintf("../codeexport/%s/tmp/coriolisvec_para_actcoord_par%d_matlab.m", robot_name, codegen_dynpar), codegen_opt);
  end if:
  if codeexport_grav then
    printf("%s. Beginne Code-Export Gravitations-Vektor in Antriebs-Koordinaten.\n", FormatTime("%Y-%m-%d %H:%M:%S")):
    MatlabExport(gGes_qa, sprintf("../codeexport/%s/tmp/gravvec_para_actcoord_par%d_matlab.m", robot_name, codegen_dynpar), codegen_opt);
  end if:
end if:

