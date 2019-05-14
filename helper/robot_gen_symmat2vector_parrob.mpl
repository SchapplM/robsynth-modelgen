
# Konvertierung Vektor zu symmetrischer Matrix
# Einleitung
# Generiere die Umwandlung von Vektor mit oberem rechten Teil einer Symmetrischen Matrix zur Matrix selbst f�r die Dimensionen des Roboters.
# Diese Umwandlung verursacht weniger Rechenoperationen als die Verwendung der numerischen Funktion vec2symmat in Matlab, da dort jedes Mal die Indizes beim Zusammenbauen der Matrix getestet werden m�ssen.
# Autor
# Tim Job (Studienarbeit bei Moritz Schappler), 2019-4
# Moritz Schappler, moritz.schappler@imes.uni-hannover.de
# (C) Institut f�r Mechatronische Systeme, Universit�t Hannover
# Initialization
interface(warnlevel=0): # Unterdr�cke die folgende Warnung.
restart: # Gibt eine Warnung, wenn �ber Terminal-Maple mit read gestartet wird.
interface(warnlevel=3):
with(LinearAlgebra):
with(ArrayTools):
with(codegen):
with(CodeGeneration):
with(StringTools):
# Einstellungen f�r Code-Export: Optimierungsgrad (2=h�chster) und Aktivierung jedes Terms.
read "../helper/proc_MatlabExport":
read "../helper/proc_vector2mat":
read "../robot_codegen_definitions/robot_env_par":
read sprintf("../codeexport/%s/tmp/para_definitions", robot_name):
printf("Generiere Symmat2Vector-Funktionen f�r %s\n", robot_name):
codegen_opt := 0: # Soll nicht von Einstellung in robot_env �berschrieben werden.
;
# Funktion symmat2vector f�r den Roboter definieren
# Erstelle eine Dummy-Variable (mv), die als tempor�re Variable in Matlab dient (zum Zusammensetzen der Matrix).
clear('mv'):
M_NX:= vec2mat(mv, NX):
# Warnungen bei Code-Generierung unterdr�cken. Die Meldung das der Ausdruck mv() in Matlab nicht bekannt ist, spielt keine Rolle, da diese Variable nach dem Einsetzen des Codes vorher definiert sein wird.
interface(warnlevel=0):
MatlabExport(M_NX, sprintf("../codeexport/%s/tmp/vec2mat_%d_matlab.m", robot_name, NX), codegen_opt):

