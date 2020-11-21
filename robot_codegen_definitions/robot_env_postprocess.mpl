
# Postprocess Definition File for Code Generation
# Init
# Nachverarbeitung der Datei robot_env. Falls dort mit Maple-Programmcode Teile der DH-Parameter definiert werden, wird diese Datei ausgewertet und die Variablen neu gespeichert.
# Benötigt, da in robot_codegen_tmpvar_bash.sh ein definiertes Muster für die robot_env erwartet wird.
# 
# Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-11
# (C) Institut fuer Mechatronische Systeme, Leibniz Universitaet Hannover
interface(warnlevel=0): # Unterdrücke die folgende Warnung.
restart: # Gibt eine Warnung, wenn über Terminal-Maple mit read gestartet wird.
interface(warnlevel=3):
# Lese Umgebungsvariable für Codegenerierung.
read "../robot_codegen_definitions/robot_env":
printf("Nachverarbeitung der Eingabedatei für %s\n",robot_name):

# Postprocess
# Speichere die Variablen neu ab
NQJ := NQJ:
NJ := NJ:
robot_name := robot_name:
save NQJ, NJ, robot_name, "../robot_codegen_definitions/robot_env2":

