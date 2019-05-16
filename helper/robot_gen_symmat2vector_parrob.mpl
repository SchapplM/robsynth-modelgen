
# Konvertierung Vektor zu symmetrischer Matrix
# Einleitung
# Generiere die Umwandlung von Vektor mit oberem rechten Teil einer Symmetrischen Matrix zur Matrix selbst für die Dimensionen des Roboters.
# Diese Umwandlung verursacht weniger Rechenoperationen als die Verwendung der numerischen Funktion vec2symmat in Matlab, da dort jedes Mal die Indizes beim Zusammenbauen der Matrix getestet werden müssen.
# Autor
# Tim Job (Studienarbeit bei Moritz Schappler), 2019-4
# Moritz Schappler, moritz.schappler@imes.uni-hannover.de
# (C) Institut für Mechatronische Systeme, Universität Hannover
# Initialization
interface(warnlevel=0): # Unterdrücke die folgende Warnung.
restart: # Gibt eine Warnung, wenn über Terminal-Maple mit read gestartet wird.
interface(warnlevel=3):
with(LinearAlgebra):
with(ArrayTools):
with(codegen):
with(CodeGeneration):
with(StringTools):
# Einstellungen für Code-Export: Optimierungsgrad (2=höchster) und Aktivierung jedes Terms.
read "../helper/proc_MatlabExport":
read "../helper/proc_vector2mat":
read "../robot_codegen_definitions/robot_env_par":
read sprintf("../codeexport/%s/tmp/para_definitions", robot_name):
printf("Generiere Symmat2Vector-Funktionen für %s\n", robot_name):
codegen_opt := 0: # Soll nicht von Einstellung in robot_env überschrieben werden.
;
# Funktion symmat2vector für den Roboter definieren
# Erstelle eine Dummy-Variable (mv), die als temporäre Variable in Matlab dient (zum Zusammensetzen der Matrix).
clear('mv'):
M_NX:= vec2mat(mv, NX):
# Warnungen bei Code-Generierung unterdrücken. Die Meldung das der Ausdruck mv() in Matlab nicht bekannt ist, spielt keine Rolle, da diese Variable nach dem Einsetzen des Codes vorher definiert sein wird.
interface(warnlevel=0):
MatlabExport(M_NX, sprintf("../codeexport/%s/tmp/vec2mat_%d_matlab.m", robot_name, NX), codegen_opt):

