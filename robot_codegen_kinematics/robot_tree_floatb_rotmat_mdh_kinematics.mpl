
# Forward Kinematics for Robot based on MDH frames
# Introduction
# Direkte Kinematik basierend auf MDH-Parametern berechnen
# 
# Dateiname:
# robot -> Berechnung für allgemeinen Roboter
# tree -> Berechnung für eine beliebige Baumstruktur (ohne Schleifen)
# floatb -> floating base wird durch base twist (Geschwindigkeit der Basis) oder vollständige Orientierung (Euler-Winkel) berücksichtigt
# rotmat -> Kinematik wird mit Rotationsmatrizen berechnet
# mdh_kinematics -> Berechnung der Vorwärtskinematik mit modifizierten DH-Parametern nach [KhalilKle1986]
# 
# Prinzip:
# Berechne die direkte Kinematik.
# Zusätzlich können kinematische Zwangsbedingungen direkt berücksichtigt werden. Diese sorgen dafür, dass MDH-Winkel durch analytische Ausdrücke verallgemeinerter Koordinaten ersetzt werden.
# Authors
# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut fuer Regelungstechnik, Leibniz Universitaet Hannover
# Sources
# [KhalilKle1986] Khalil, W. & Kleinfinger, J.: A new geometric notation for open and closed-loop robots (1986)
# [KhalilDombre2002] Modeling, Identification and Control of Robots
# [Ortmaier2014] Vorlesungsskript Robotik I
# Initialization
interface(warnlevel=0): # Unterdrücke die folgende Warnung.
restart: # Gibt eine Warnung, wenn über Terminal-Maple mit read gestartet wird.
interface(warnlevel=3):
with(LinearAlgebra):
with(ArrayTools):
with(codegen):
with(CodeGeneration):
with(StringTools):
codegen_act := true:
codegen_debug := false:
codegen_opt := 2:
# Code-Optimierung für die Kinematik (zusammenfassung paralleler Achsen).
# Ist für einige Systeme vorteilhaft (serielle Ketten mit parallelen Achsen), für andere nicht (z.B. Baumstrukturen mit parallelen Achsen. Hier kann die Code-Optimierung mehr aus nicht trigonometrisch optimierten Termen herausholen).
# Die Terme werden aber auf jeden Fall übersichtlicher.
codegen_kinematics_opt := true:
# Substitutionsreihenfolge von kinematischen Zwangsbedingungen (falls vorhanden) einstellen.
# 1: Ersetzung bereits in Einzel-Transformationsmatrizen (dann ist die Zusammenfassung für mehrere Achsen aus codegen_kinematics_opt nicht mehr so leicht möglich). Muss 1 sein für trigonometrische Ersetzung.
# 2: Ersetzung erst nach der Zusammenfassung der Einzel-Transformationsmatrizen zu (kumulierten) Gesamt-Transformationsmatrizen
codegen_kinematics_subsorder := 1:
read "../helper/proc_convert_s_t":
read "../helper/proc_convert_t_s": 
read "../helper/proc_MatlabExport":
read "../transformation/proc_rotx": 
read "../transformation/proc_roty": 
read "../transformation/proc_rotz": 
read "../transformation/proc_trotx": 
read "../transformation/proc_troty": 
read "../transformation/proc_trotz": 
read "../transformation/proc_transl": 
read "../transformation/proc_trafo_mdh": 
read "../transformation/proc_rpy2r": 
read "../transformation/proc_rpy2tr": 
read "../robot_codegen_definitions/robot_env":
printf("Generiere Kinematik für %s\n", robot_name):
read sprintf("../codeexport/%s/tmp/tree_floatb_definitions", robot_name):
# Kinematische Zwangsbedingungen
# Lade Ausdrücke für kinematische Zwangsbedingungen (Verknüpfung von MDH-Gelenkwinkeln durch verallgemeinerte Koordinaten)
# Lese Variablen: kintmp_subsexp, kintmp_qs, kintmp_qt
kin_constraints_exist := false:
read "../robot_codegen_constraints/proc_subs_kintmp_exp":
constrfile := sprintf("../codeexport/%s/tmp/kinematic_constraints_maple_inert.m", robot_name):
if FileTools[Exists](constrfile) then
  read constrfile:
end if:
if kin_constraints_exist = true then:
  kintmp_qs := kintmp_qs: # gelesene Variable sonst nicht sichtbar
  kintmp_qt := kintmp_qt: # gelesene Variable sonst nicht sichtbar
  kintmp_subsexp := kintmp_subsexp:
  printf("Kinematische Zwangsbedingungen gelesen.\n"):
else
  kintmp_qs := Matrix(1,1):
  kintmp_qt := Matrix(1,1):
  kintmp_subsexp := Matrix(1,2):
  kin_constraints_exist := false:
  # Zwangsbedingungen neu speichern, damit diese auch für andere Skripte verfügbar sind (als dummy-Variablen).
  save kin_constraints_exist, kintmp_qs, kintmp_qt, kintmp_subsexp, sprintf("../codeexport/%s/tmp/kinematic_constraints_maple_inert.m", robot_name):
end if:
# Calculate Forward Kinematics (Single-Joint Transformation)
# Trf is the Matrix of Transformation from i-1 to i
# Trf_c is the cummulated Matrix of Transformation from 0 to i
Trf := Matrix(4, 4, NJ): # Diese Initialisierung bringt nichts (initialisiert nur 4x4-Matrix)
;
for i from 1 to NJ do 
  Trf(1 .. 4, 1 .. 4, i) := Matrix(4,4):
end do:
for i from 1 to NJ do 
  Trf(1 .. 4, 1 .. 4, i) := trafo_mdh_full(alpha[i,1], a[i,1], theta[i,1], d[i,1], beta[i,1], b[i,1]):
end do:
# Kinematische Zwangsbedingungen ersetzen (vor Berechnung der Gesamt-Transformation)
# Substituiere allgemeine Ausdrücke der Winkel der Parallelstruktur mit kinematischen Zwangsbedingungen in Abhängigkeit der Haupt-Gelenkwinkel
if kin_constraints_exist and codegen_kinematics_subsorder = 1 then
  for i from 1 to NJ do # Index über Transformationsmatrizen aller Körper
    for ix from 1 to 4 do # Index über Zeilen der Transformationsmatrizen
      for iy from 1 to 4 do # Index über Spalten der Transformationsmatrizen
        # Substituiere Zeitvariablen
        Trf(ix, iy, i) := convert_t_s( Trf(ix, iy, i) ):
        # Substituiere sin und cos der Winkel (einfachere Ausdrücke)
        Trf(ix, iy, i) := subs_kintmp_exp(Trf(ix, iy, i)):
        # Substituiere die verbleibenden Winkel direkt (einige Winkel sind nicht in den Ersetzungsausdrücken enthalten, da sie keine problematischen arctan-Ausdrücke enthalten.
        for jj from 1 to RowDimension(kintmp_qt) do # Index über zu ersetzende Winkel
          Trf(ix, iy, i) := subs( { kintmp_s(jj, 1) = kintmp_qs(jj, 1) }, Trf(ix, iy, i) ): 
        end do:
        Trf(ix, iy, i) := convert_s_t( Trf(ix, iy, i) ):
      end do:
    end do:
  end do:
  printf("Ersetzungen der MDH-Parameter mit Ergebnissen der Parallelstruktur in verallgemeinerten Koordinaten erfolgreich (vor Berechnung der Gesamt-Transformation).\n"):
end if:
# Calculate Forward Kinematics (Multi-Joint Transformation)
Trf_c := Matrix(4, 4, NJ+1): # Diese Initialisierung bringt nichts (initialisiert nur 4x4-Matrix)
for i from 1 to NJ+1 do 
  Trf_c(1 .. 4, 1 .. 4, i) := Matrix(4,4): # Vollständige Initialisierung
end do:
# Basis-Transformation: Unterschiedliche Darstellungsmethoden. Führen zu unterschiedlichen verallgemeinerten Koordinaten.
if base_method_name = "twist" then:
  Trf_c(1 .. 4, 1 .. 4, 1) := transl(X_base_t[1..3,1]):
end:
if base_method_name = "eulxyz" then:
  Trf_c(1 .. 4, 1 .. 4, 1) := transl(X_base_t[1..3,1]).rpy2tr(X_base_t[4,1], X_base_t[5,1], X_base_t[6,1]):
end:
printf("Nutze die Methode %s für die Basis-Orientierung\n", base_method_name):
# Kinematik aller Körper mit MDH-Ansatz Bestimmen. [KhalilKle1986].

# Einfache Kinematik: Multiplikation an vorherige Transformationsmatrix
if not(codegen_kinematics_opt) then
  for i from 1 to NJ do 
    # Index des vorherigen Koordinatensystems
    j := v(i)+1:
    Trf_c(1 .. 4, 1 .. 4, i+1) := Multiply(Trf_c(1 .. 4, 1 .. 4, j), Trf(1 .. 4, 1 .. 4, i)):
  end do:
end if:
#Trf_c_orig := copy(Trf_c): # Debug-Ausdruck zum Vergleich der obigen mit der unteren Methode
;
# Optimierung der Terme: Summentheorem für Drehungen um aufeinanderfolgende Achsen.
# Bei Verwendung dieser Optimierung funktioniert die Substitution mit trigonometrischen Ausdrücken von abhängigen Gelenken eventuell anders.
# Die Funktionsweise kann mit aktivierten print-Befehlen nachvollzogen werden.
if codegen_kinematics_opt then
  for i from 1 to NJ do
    #printf("----------------------\ni=%d\n",i):
    # Bestimme Vorwärtskinematik für das Koordinatensystem i (0=Basis)
    # Gehe solande rückwärts, wie die Gelenkachsen parallel sind
    j := i: # Lauf-Index über Vorgänger-Elemente (0=Basis)
    j2 := 0:# Index des Segmentes (0=Basis), das vor der Kette von parallelen Achsen ist
    Trf_tmp := IdentityMatrix(4,4); # Transformation von diesem Segment zum aktuellen
    Kette_akt := []:
    #printf("Trf_tmp neu initialisiert:\n"):
    #print(Trf_tmp);
    for k from 1 to NJ do # Schleife mit Dummy-Länge. Wird abgebrochen, falls beendet.
    	 #printf("j=%d\n",j):
    	 Kette_akt := [j, op(Kette_akt)]:
      Trf_tmp := simplify(combine( Matrix(Trf(1 .. 4, 1 .. 4, j) . Trf_tmp) )): # Additionstheorem für Drehung um parallele Achsen
    	 #printf("Trf_tmp aktualisiert mit Trf %d. Aktuelle Kette: %s\n", j, convert(Kette_akt, string)):
    	 #print(Trf_tmp);
      if not(alpha(j) = 0) then
        # Die vorherige Achse ist nicht parallel zu dieser
        # Weitere Vereinfachungen ergeben keinen Sinn
        j2 := v(j):
        #printf("alpha ungleich null. Abbruch j2=%d.\n", j2):
        break:
      #else
      #  printf("alpha ist Null. Vorherige Achse ist parallel\n"):
      end if:
      if v(j) = 0 then
        break: # Vorgänger ist Basis. Keine weitere Untersuchung
      end if:
      #printf("v(%d)=%d\n", j, v(j)):
      j := v(j): # Nehme das Vorgänger-Segment
    end do:
    Trf_c(1 .. 4, 1 .. 4, i+1) := Matrix(Trf_c(1 .. 4, 1 .. 4, j2+1)) . Trf_tmp:
    #printf("Trf_tmp an Trf_c angehängt (Eintrag zu Körper %d. %d -> %d mit Kette %s)\n", i, j2, i, convert(Kette_akt, string)):
    #print(Trf_c(1 .. 4, 1 .. 4, i+1)):
  end do;
end if;
# Kinematische Zwangsbedingungen ersetzen (nach Berechnung der Gesamt-Transformation)
# Substituiere allgemeine Ausdrücke der Winkel der Parallelstruktur mit kinematischen Zwangsbedingungen in Abhängigkeit der Haupt-Gelenkwinkel
# Da die Gesamt-Transformation Trf_c an dieser Stelle schon berechnet wurde, müssen die Ausdrücke hier in Trf und Trf_c ersetzt werden.
if kin_constraints_exist and codegen_kinematics_subsorder = 2 then
  for i from 1 to NJ do # Index über Transformationsmatrizen aller Körper
    for ix from 1 to 4 do # Index über Zeilen der Transformationsmatrizen
      for iy from 1 to 4 do # Index über Spalten der Transformationsmatrizen
        # Substituiere Zeitvariablen
        Trf(  ix, iy, i)   := convert_t_s( Trf(  ix, iy, i) ):
        Trf_c(ix, iy, i+1) := convert_t_s( Trf_c(ix, iy, i+1) ):
        # Substituiere sin und cos der Winkel (einfachere Ausdrücke)
        Trf(ix,   iy, i)   := subs_kintmp_exp(Trf(  ix, iy, i)):
        Trf_c(ix, iy, i+1) := subs_kintmp_exp(Trf_c(ix, iy, i+1)):
        # Substituiere die verbleibenden Winkel direkt (einige Winkel sind nicht in den Ersetzungsausdrücken enthalten, da sie keine problematischen arctan-Ausdrücke enthalten.
        for jj from 1 to RowDimension(kintmp_qt) do # Index über zu ersetzende Winkel
          Trf(ix,   iy, i)   := subs( { kintmp_s(jj, 1) = kintmp_qs(jj, 1) }, Trf(  ix, iy, i) ):
          Trf_c(ix, iy, i+1) := subs( { kintmp_s(jj, 1) = kintmp_qs(jj, 1) }, Trf_c(ix, iy, i+1) ): 
        end do:
        Trf(ix,   iy, i)   := convert_s_t( Trf(  ix, iy, i) ):
        Trf_c(ix, iy, i+1) := convert_s_t( Trf_c(ix, iy, i+1) ):
      end do:
    end do:
  end do:
  printf("Ersetzungen der MDH-Parameter mit Ergebnissen der Parallelstruktur in verallgemeinerten Koordinaten erfolgreich (nach Berechnung der Gesamt-Transformation).\n"):
end if:
# Export
# Maple-Export
save Trf, Trf_c, sprintf("../codeexport/%s/tmp/kinematics_floatb_%s_rotmat_maple.m", robot_name, base_method_name):
# Export des symbolischen Ausdrucks für alle kumulierten Transformationsmatrizen auf einmal.
Trf_c_Export := Matrix((NJ+1)*4, 4):
for i from 1 to NJ+1 do 
  Trf_c_Export((i-1)*4+1 .. 4*i, 1..4) := Trf_c(1..4, 1..4, i):
end do:
if codegen_act then
  MatlabExport(convert_t_s(Trf_c_Export), sprintf("../codeexport/%s/tmp/fkine_mdh_floatb_%s_rotmat_matlab.m", robot_name, base_method_name), codegen_opt):
end if:
# Export des symbolischen Ausdrucks für alle Gelenk-Transformationsmatrizen auf einmal.
Trf_Export := Matrix((NJ)*4, 4):
for i from 1 to NJ do 
  Trf_Export((i-1)*4+1 .. 4*i, 1..4) := Trf(1..4, 1..4, i):
end do:
if codegen_act then
  MatlabExport(convert_t_s(Trf_Export), sprintf("../codeexport/%s/tmp/joint_transformation_mdh_rotmat_matlab.m", robot_name), codegen_opt):
end if:
# Export ohne letzte Zeile
Trf_Export_m := Matrix((NJ)*3, 4):
for i from 1 to NL do 
  Trf_Export_m((i-1)*3+1 .. 3*i, 1..4) := Trf_c(1..3, 1..4, i):
end do:
if codegen_act then
  MatlabExport(convert_t_s(Trf_Export_m), sprintf("../codeexport/%s/tmp/joint_transformation_mdh_rotmat_m_matlab.m", robot_name), codegen_opt):
end if:
# Export des symbolischen Ausdrucks für jede Transformationsmatrix einzeln
for i from 1 to NJ+1 do
  if codegen_act and codegen_debug then
    MatlabExport(convert_t_s(Trf_c(1 .. 4, 1 .. 4, i)), sprintf("../codeexport/%s/tmp/fkine_%d_floatb_%s_rotmat_matlab.m", robot_name, i, base_method_name), codegen_opt):
  end if:
end do:

