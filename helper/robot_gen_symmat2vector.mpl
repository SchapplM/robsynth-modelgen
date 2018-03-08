
# Konvertierung Vektor zu symmetrischer Matrix
# Einleitung
# Generiere die Umwandlung von Vektor mit oberem rechten Teil einer Symmetrischen Matrix zur Matrix selbst für die Dimensionen des Roboters.
# Diese Umwandlung verursacht weniger Rechenoperationen als die Verwendung der numerischen Funktion vec2symmat in Matlab, da dort jedes Mal die Indizes beim Zusammenbauen der Matrix getestet werden müssen.
# Autor
# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-12
# (C) Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
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
codegen_opt := 0:
read "../helper/proc_MatlabExport":
read "../helper/proc_vector2symmat":
read "../robot_codegen_definitions/robot_env":
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name):
printf("Generiere Symmat2Vector-Funktionen für %s\n", robot_name):
# Funktion symmat2vector für den Roboter definieren
# Erstelle eine Dummy-Variable (mv), die als temporäre Variable in Matlab dient (zum Zusammensetzen der Matrix).
clear('mv'):
M_NQ:= vec2symmat(mv, NQ):
M_NQJ:= vec2symmat(mv, NQJ):
M_6:= vec2symmat(mv, 6):
# Warnungen bei Code-Generierung unterdrücken. Die Meldung das der Ausdruck mv() in Matlab nicht bekannt ist, spielt keine Rolle, da diese Variable nach dem Einsetzen des Codes vorher definiert sein wird.
interface(warnlevel=0):
MatlabExport(M_NQ, sprintf("../codeexport/%s/tmp/vec2symmat_%d_matlab.m", robot_name, NQ), codegen_opt):
MatlabExport(M_NQJ, sprintf("../codeexport/%s/tmp/vec2symmat_%d_matlab.m", robot_name, NQJ), codegen_opt):
MatlabExport(M_6, sprintf("../codeexport/%s/tmp/vec2symmat_%d_matlab.m", robot_name, 6), codegen_opt):

