#!/bin/bash -e
# Erstelle temporäre Variablen mit Code-Schnipseln, die für die Erzeugung von Matlab-Code benötigt wird.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

repo_pfad=$(pwd)/../
tmp_pfad=$repo_pfad/workdir/tmp
source $repo_pfad/robot_codegen_definitions/robot_env.sh

# Schnipsel für Matlab-varpar-Dateien vorbereiten
# Gelenkwinkel
echo "" > $tmp_pfad/robot_matlabtmp_qJ.m
for (( i=1; i<=$robot_NQJ; i++ ))
do
	echo "qJ${i}s = qJ(${i});" >> $tmp_pfad/robot_matlabtmp_qJ.m
done

# Gelenkwinkelgeschwindigkeit
echo "" > $tmp_pfad/robot_matlabtmp_qJD.m
for (( i=1; i<=$robot_NQJ; i++ ))
do
	echo "qJD${i}s = qJD(${i});" >> $tmp_pfad/robot_matlabtmp_qJD.m
done

# Gelenkwinkelbeschleunigung
echo "" > $tmp_pfad/robot_matlabtmp_qJDD.m
for (( i=1; i<=$robot_NQJ; i++ ))
do
	echo "qJDD${i}s = qJDD(${i});" >> $tmp_pfad/robot_matlabtmp_qJDD.m
done

# Kinematikparameter (alle als Vektor): MDH-Parameter und Parameter für Zwangsbedingungen
echo "" > $tmp_pfad/robot_matlabtmp_par_KP.m
i=0;
for Kp in $robot_KP; do
  i=$((i+1));          
  echo "$Kp = pkin($i);" >> $tmp_pfad/robot_matlabtmp_par_KP.m
done

# Kinematik MDH (separat, wird nur noch von wenigen Funktionen gebraucht)
echo "" > $tmp_pfad/robot_matlabtmp_par_mdh.m
for (( i=1; i<=$robot_NJ; i++ ))
do
	echo "a${i} = a_mdh(${i});" >> $tmp_pfad/robot_matlabtmp_par_mdh.m
done
for (( i=1; i<=$robot_NJ; i++ ))
do
	echo "alpha${i} = alpha_mdh(${i});" >> $tmp_pfad/robot_matlabtmp_par_mdh.m
done
for (( i=1; i<=$robot_NJ; i++ ))
do
	echo "d${i} = d_mdh(${i});" >> $tmp_pfad/robot_matlabtmp_par_mdh.m
done
for (( i=1; i<=$robot_NJ; i++ ))
do
	echo "qoffset${i} = q_offset_mdh(${i});" >> $tmp_pfad/robot_matlabtmp_par_mdh.m
done
for (( i=1; i<=$robot_NJ; i++ ))
do
	echo "b${i} = b_mdh(${i});" >> $tmp_pfad/robot_matlabtmp_par_mdh.m
done
for (( i=1; i<=$robot_NJ; i++ ))
do
	echo "beta${i} = beta_mdh(${i});" >> $tmp_pfad/robot_matlabtmp_par_mdh.m
done

# Dynamikparameter.
# Das Matlab-Array beginnt bei 1 zu zählen. Die Maple-Variablen beginnen in der Benennung bei 0 (MDH-Notation)
echo "" > $tmp_pfad/robot_matlabtmp_par_m.m
for (( i=0; i<$robot_NL; i++ ))
do
  j=$((i+1)) # Index für Matlab-Variablen (beginnen bei 1)
	echo "M${i} = m(${j});" >> $tmp_pfad/robot_matlabtmp_par_m.m
done

echo "" > $tmp_pfad/robot_matlabtmp_par_rcom.m
for (( i=0; i<$robot_NL; i++ ))
do
  j=$((i+1)) # Index für Matlab-Variablen (beginnen bei 1)
	echo "SX${i} = rSges(${j},1);" >> $tmp_pfad/robot_matlabtmp_par_rcom.m
	echo "SY${i} = rSges(${j},2);" >> $tmp_pfad/robot_matlabtmp_par_rcom.m
	echo "SZ${i} = rSges(${j},3);" >> $tmp_pfad/robot_matlabtmp_par_rcom.m
done

echo "" > $tmp_pfad/robot_matlabtmp_par_mrcom.m
for (( i=0; i<$robot_NL; i++ ))
do
  j=$((i+1)) # Index für Matlab-Variablen (beginnen bei 1)
	echo "MX${i} = mrSges(${j},1);" >> $tmp_pfad/robot_matlabtmp_par_mrcom.m
	echo "MY${i} = mrSges(${j},2);" >> $tmp_pfad/robot_matlabtmp_par_mrcom.m
	echo "MZ${i} = mrSges(${j},3);" >> $tmp_pfad/robot_matlabtmp_par_mrcom.m
done

echo "" > $tmp_pfad/robot_matlabtmp_par_Ic.m
for (( i=0; i<$robot_NL; i++ ))
do
  j=$((i+1)) # Index für Matlab-Variablen (beginnen bei 1)
	echo "XXC${i} = Icges(${j},1);" >> $tmp_pfad/robot_matlabtmp_par_Ic.m
	echo "XYC${i} = Icges(${j},4);" >> $tmp_pfad/robot_matlabtmp_par_Ic.m
	echo "XZC${i} = Icges(${j},5);" >> $tmp_pfad/robot_matlabtmp_par_Ic.m
	echo "YYC${i} = Icges(${j},2);" >> $tmp_pfad/robot_matlabtmp_par_Ic.m
	echo "YZC${i} = Icges(${j},6);" >> $tmp_pfad/robot_matlabtmp_par_Ic.m
	echo "ZZC${i} = Icges(${j},3);" >> $tmp_pfad/robot_matlabtmp_par_Ic.m
done

echo "" > $tmp_pfad/robot_matlabtmp_par_If.m
for (( i=0; i<$robot_NL; i++ ))
do
  j=$((i+1)) # Index für Matlab-Variablen (beginnen bei 1)
	echo "XX${i} = Ifges(${j},1);" >> $tmp_pfad/robot_matlabtmp_par_If.m
	echo "XY${i} = Ifges(${j},4);" >> $tmp_pfad/robot_matlabtmp_par_If.m
	echo "XZ${i} = Ifges(${j},5);" >> $tmp_pfad/robot_matlabtmp_par_If.m
	echo "YY${i} = Ifges(${j},2);" >> $tmp_pfad/robot_matlabtmp_par_If.m
	echo "YZ${i} = Ifges(${j},6);" >> $tmp_pfad/robot_matlabtmp_par_If.m
	echo "ZZ${i} = Ifges(${j},3);" >> $tmp_pfad/robot_matlabtmp_par_If.m
done

# Alle Dynamikparameter als Vektor (Minimale Form, Fixed Base)
echo "" > $tmp_pfad/robot_matlabtmp_par_MDPFIXB.m
for (( i=1; i<=$robot_NMPVFIXB; i++ )); do      
  echo "MDP${i} = MDP(${i});" >> $tmp_pfad/robot_matlabtmp_par_MDPFIXB.m
done

# Gravitationsvektor
echo "" > $tmp_pfad/robot_matlabtmp_g.m
for (( i=1; i<=3; i++ ))
do
	echo "g${i} = g(${i});" >> $tmp_pfad/robot_matlabtmp_g.m
done

# Basisposition
echo "" > $tmp_pfad/robot_matlabtmp_rB.m
echo "rxs_base = r_base(1);" >> $tmp_pfad/robot_matlabtmp_rB.m
echo "rys_base = r_base(2);" >> $tmp_pfad/robot_matlabtmp_rB.m
echo "rzs_base = r_base(3);" >> $tmp_pfad/robot_matlabtmp_rB.m

# Basisorientierung (Euler-XYZ)
echo "" > $tmp_pfad/robot_matlabtmp_phiB.m
echo "alphaxs_base = phi_base(1);" >> $tmp_pfad/robot_matlabtmp_phiB.m
echo "betays_base = phi_base(2);" >> $tmp_pfad/robot_matlabtmp_phiB.m
echo "gammazs_base = phi_base(3);" >> $tmp_pfad/robot_matlabtmp_phiB.m

# Basisgeschwindigkeit
echo "" > $tmp_pfad/robot_matlabtmp_vB.m
echo "vxs_base = V_base(1);" >> $tmp_pfad/robot_matlabtmp_vB.m
echo "vys_base = V_base(2);" >> $tmp_pfad/robot_matlabtmp_vB.m
echo "vzs_base = V_base(3);" >> $tmp_pfad/robot_matlabtmp_vB.m
echo "omegaxs_base = V_base(4);" >> $tmp_pfad/robot_matlabtmp_vB.m
echo "omegays_base = V_base(5);" >> $tmp_pfad/robot_matlabtmp_vB.m
echo "omegazs_base = V_base(6);" >> $tmp_pfad/robot_matlabtmp_vB.m

# Basisbeschleunigung
echo "" > $tmp_pfad/robot_matlabtmp_vBD.m
echo "vDxs_base = A_base(1);" >> $tmp_pfad/robot_matlabtmp_aB.m
echo "vDys_base = A_base(2);" >> $tmp_pfad/robot_matlabtmp_aB.m
echo "vDzs_base = A_base(3);" >> $tmp_pfad/robot_matlabtmp_aB.m
echo "omegaDxs_base = A_base(4);" >> $tmp_pfad/robot_matlabtmp_aB.m
echo "omegaDys_base = A_base(5);" >> $tmp_pfad/robot_matlabtmp_aB.m
echo "omegaDzs_base = A_base(6);" >> $tmp_pfad/robot_matlabtmp_aB.m

# Basisgeschwindigkeit (Ableitung der Euler-Winkel)
echo "" > $tmp_pfad/robot_matlabtmp_xDB.m
echo "vxs_base = xD_base(1);" >> $tmp_pfad/robot_matlabtmp_xDB.m
echo "vys_base = xD_base(2);" >> $tmp_pfad/robot_matlabtmp_xDB.m
echo "vzs_base = xD_base(3);" >> $tmp_pfad/robot_matlabtmp_xDB.m
echo "alphaDx_base = xD_base(4);" >> $tmp_pfad/robot_matlabtmp_xDB.m
echo "betaDy_base = xD_base(5);" >> $tmp_pfad/robot_matlabtmp_xDB.m
echo "gammaDz_base = xD_base(6);" >> $tmp_pfad/robot_matlabtmp_xDB.m

# Basisbeschleunigung (Zweite Ableitung der Euler-Winkel)
echo "" > $tmp_pfad/robot_matlabtmp_xDDB.m
echo "vDxs_base = xDD_base(1);" >> $tmp_pfad/robot_matlabtmp_xDDB.m
echo "vDys_base = xDD_base(2);" >> $tmp_pfad/robot_matlabtmp_xDDB.m
echo "vDzs_base = xDD_base(3);" >> $tmp_pfad/robot_matlabtmp_xDDB.m
echo "alphaDDx_base = xDD_base(4);" >> $tmp_pfad/robot_matlabtmp_xDDB.m
echo "betaDDy_base = xDD_base(5);" >> $tmp_pfad/robot_matlabtmp_xDDB.m
echo "gammaDDz_base = xDD_base(6);" >> $tmp_pfad/robot_matlabtmp_xDDB.m

# Alle Code-Schnipsel für Variablendefinitionen in eine Datei schreiben
zd=$tmp_pfad/robot_matlabtmp_all.m
echo "" > $zd
for f in `find $tmp_pfad -name "robot_matlabtmp_*.m" ! -name "robot_matlabtmp_all.m"`; do
  cat $f >> $zd
done
