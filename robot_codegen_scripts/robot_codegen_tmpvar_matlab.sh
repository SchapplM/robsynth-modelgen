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

# Basisorientierung (EulerXYZ / RPY)
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

# Parallele Roboter
# Gelenkwinkel
echo "" > $tmp_pfad/robot_matlabtmp_qJ_parallel.m
for (( leg=1; leg<=$parallel_NLEGS; leg++ ))
do
	for (( i=1 ; i<=$parallel_NQJ_leg ; i++ ))
	do
		diffi=$(( parallel_NQJ_leg - i ))
		tmp=$(( leg * parallel_NQJ_leg ))
		count=$(( tmp - diffi ))
		echo "qJ${count}s = qJ(${i},${leg});" >> $tmp_pfad/robot_matlabtmp_qJ_parallel.m
	done
done

# Plattform-Koordinaten
echo "" > $tmp_pfad/robot_matlabtmp_xP.m
for (( i=1 ; i<=$parallel_NX ; i++ ))
do
	echo "x_all(${i}) = xP(${i});" >> $tmp_pfad/robot_matlabtmp_xP.m
done

# Plattform-Geschwindigkeiten
echo "" > $tmp_pfad/robot_matlabtmp_xPD.m
for (( i=1 ; i<=$parallel_NX ; i++ ))
do
	echo "Dx_all(${i}) = xPD(${i});" >> $tmp_pfad/robot_matlabtmp_xPD.m
done

# Plattform-Beschleunigungen
echo "" > $tmp_pfad/robot_matlabtmp_xPDD.m
for (( i=1 ; i<=$parallel_NX ; i++ ))
do
	echo "DDx_all(${i}) = xPDD(${i});" >> $tmp_pfad/robot_matlabtmp_xPDD.m
done

# # Kinematikparameter (alle als Vektor): MDH-Parameter und Parameter für Zwangsbedingungen
# echo "" > $tmp_pfad/robot_matlabtmp_par_KP.m
# i=0;
# for Kp in $robot_KP; do
  # i=$((i+1));          
  # echo "$Kp = pkin($i);" >> $tmp_pfad/robot_matlabtmp_par_KP_parallel.m
# done

# Dynamikparameter.
# Das Matlab-Array beginnt bei 1 zu zählen. Die Maple-Variablen beginnen in der Benennung bei 0 (MDH-Notation)
echo "" > $tmp_pfad/robot_matlabtmp_par_m_parallel.m
for (( i=0; i<$parallel_NQJ_leg; i++ ))
do
  j=$((i+1)) # Index für Matlab-Variablen (beginnen bei 1)
	echo "M${j} = m(${j});" >> $tmp_pfad/robot_matlabtmp_par_m_parallel.m
done
j=$((parallel_NQJ_leg+1))
echo "mE = m(${j});" >> $tmp_pfad/robot_matlabtmp_par_m_parallel.m

echo "" > $tmp_pfad/robot_matlabtmp_par_rcom_parallel.m
for (( i=0; i<$parallel_NQJ_leg; i++ ))
do
  j=$((i+1)) # Index für Matlab-Variablen (beginnen bei 1)
	echo "SX${j} = rSges(${j},1);" >> $tmp_pfad/robot_matlabtmp_par_rcom_parallel.m
	echo "SY${j} = rSges(${j},2);" >> $tmp_pfad/robot_matlabtmp_par_rcom_parallel.m
	echo "SZ${j} = rSges(${j},3);" >> $tmp_pfad/robot_matlabtmp_par_rcom_parallel.m
done
j=$((parallel_NQJ_leg+1))
echo "r_sP(1) = rSges(${j},1);" >> $tmp_pfad/robot_matlabtmp_par_rcom_parallel.m
echo "r_sP(2) = rSges(${j},2);" >> $tmp_pfad/robot_matlabtmp_par_rcom_parallel.m
echo "r_sP(3) = rSges(${j},3);" >> $tmp_pfad/robot_matlabtmp_par_rcom_parallel.m

echo "" > $tmp_pfad/robot_matlabtmp_par_mrcom_parallel.m
for (( i=0; i<$parallel_NQJ_leg; i++ ))
do
  j=$((i+1)) # Index für Matlab-Variablen (beginnen bei 1)
	echo "MX${j} = mrSges(${j},1);" >> $tmp_pfad/robot_matlabtmp_par_mrcom_parallel.m
	echo "MY${j} = mrSges(${j},2);" >> $tmp_pfad/robot_matlabtmp_par_mrcom_parallel.m
	echo "MZ${j} = mrSges(${j},3);" >> $tmp_pfad/robot_matlabtmp_par_mrcom_parallel.m
done
j=$((parallel_NQJ_leg+1))
echo "s_sP(1) = mrSges(${j},1);" >> $tmp_pfad/robot_matlabtmp_par_mrcom_parallel.m
echo "s_sP(2) = mrSges(${j},2);" >> $tmp_pfad/robot_matlabtmp_par_mrcom_parallel.m
echo "s_sP(3) = mrSges(${j},3);" >> $tmp_pfad/robot_matlabtmp_par_mrcom_parallel.m

echo "" > $tmp_pfad/robot_matlabtmp_par_Ic_parallel.m
for (( i=0; i<$parallel_NQJ_leg; i++ ))
do
  j=$((i+1)) # Index für Matlab-Variablen (beginnen bei 1)
	echo "XXC${j} = Icges(${j},1);" >> $tmp_pfad/robot_matlabtmp_par_Ic_parallel.m
	echo "XYC${j} = Icges(${j},4);" >> $tmp_pfad/robot_matlabtmp_par_Ic_parallel.m
	echo "XZC${j} = Icges(${j},5);" >> $tmp_pfad/robot_matlabtmp_par_Ic_parallel.m
	echo "YYC${j} = Icges(${j},2);" >> $tmp_pfad/robot_matlabtmp_par_Ic_parallel.m
	echo "YZC${j} = Icges(${j},6);" >> $tmp_pfad/robot_matlabtmp_par_Ic_parallel.m
	echo "ZZC${j} = Icges(${j},3);" >> $tmp_pfad/robot_matlabtmp_par_Ic_parallel.m
done
j=$((parallel_NQJ_leg+1))
echo "XX = Icges(${j},1);" >> $tmp_pfad/robot_matlabtmp_par_Ic_parallel.m
echo "XY = Icges(${j},4);" >> $tmp_pfad/robot_matlabtmp_par_Ic_parallel.m
echo "XZ = Icges(${j},5);" >> $tmp_pfad/robot_matlabtmp_par_Ic_parallel.m
echo "YY = Icges(${j},2);" >> $tmp_pfad/robot_matlabtmp_par_Ic_parallel.m
echo "YZ = Icges(${j},6);" >> $tmp_pfad/robot_matlabtmp_par_Ic_parallel.m
echo "ZZ = Icges(${j},3);" >> $tmp_pfad/robot_matlabtmp_par_Ic_parallel.m
	
echo "" > $tmp_pfad/robot_matlabtmp_par_If_parallel.m
for (( i=0; i<$parallel_NQJ_leg; i++ ))
do
  j=$((i+1)) # Index für Matlab-Variablen (beginnen bei 1)
	echo "XX${j} = Ifges(${j},1);" >> $tmp_pfad/robot_matlabtmp_par_If_parallel.m
	echo "XY${j} = Ifges(${j},4);" >> $tmp_pfad/robot_matlabtmp_par_If_parallel.m
	echo "XZ${j} = Ifges(${j},5);" >> $tmp_pfad/robot_matlabtmp_par_If_parallel.m
	echo "YY${j} = Ifges(${j},2);" >> $tmp_pfad/robot_matlabtmp_par_If_parallel.m
	echo "YZ${j} = Ifges(${j},6);" >> $tmp_pfad/robot_matlabtmp_par_If_parallel.m
	echo "ZZ${j} = Ifges(${j},3);" >> $tmp_pfad/robot_matlabtmp_par_If_parallel.m
done
j=$((parallel_NQJ_leg+1))
echo "XXFP = Ifges(${j},1);" >> $tmp_pfad/robot_matlabtmp_par_If_parallel.m
echo "XYFP = Ifges(${j},4);" >> $tmp_pfad/robot_matlabtmp_par_If_parallel.m
echo "XZFP = Ifges(${j},5);" >> $tmp_pfad/robot_matlabtmp_par_If_parallel.m
echo "YYFP = Ifges(${j},2);" >> $tmp_pfad/robot_matlabtmp_par_If_parallel.m
echo "YZFP = Ifges(${j},6);" >> $tmp_pfad/robot_matlabtmp_par_If_parallel.m
echo "ZZFP = Ifges(${j},3);" >> $tmp_pfad/robot_matlabtmp_par_If_parallel.m

# Orientierungen der Beinbasis-KOs
echo "" > $tmp_pfad/robot_matlabtmp_legFrame_parallel.m
for (( i=0; i<$parallel_NLEGS; i++ ))
do
  j=$((i+1)) # Index für Matlab-Variablen (beginnen bei 1)
	echo "alphaLeg(${j}) = legFrame(${j},1);" >> $tmp_pfad/robot_matlabtmp_legFrame_parallel.m
	echo "betaLeg(${j}) = legFrame(${j},2);" >> $tmp_pfad/robot_matlabtmp_legFrame_parallel.m
	echo "gammaLeg(${j}) = legFrame(${j},3);" >> $tmp_pfad/robot_matlabtmp_legFrame_parallel.m
done

# Koppelpunktkoordinaten
echo "" > $tmp_pfad/robot_matlabtmp_par_koppelP_parallel.m
for (( l=1; l<=$parallel_NLEGS; l++ ))
do
	echo "xP(${l}) = koppelP(${l},1);" >> $tmp_pfad/robot_matlabtmp_par_koppelP_parallel.m
	echo "yP(${l}) = koppelP(${l},2);" >> $tmp_pfad/robot_matlabtmp_par_koppelP_parallel.m
	echo "zP(${l}) = koppelP(${l},3);" >> $tmp_pfad/robot_matlabtmp_par_koppelP_parallel.m
done

# Schwerpunkt Plattform-Beschleunigungen
echo "" > $tmp_pfad/robot_matlabtmp_par_rSP_parallel.m
for (( i=0; i<3; i++ ))
do
  j=$((i+1)) # Index für Matlab-Variablen (beginnen bei 1)
	echo "r_sP(${j}) = rSP(${j});" >> $tmp_pfad/robot_matlabtmp_par_rSP_parallel.m
done

# Alle Code-Schnipsel für Variablendefinitionen in eine Datei schreiben
zd=$tmp_pfad/robot_matlabtmp_all.m
echo "" > $zd
for f in `find $tmp_pfad -name "robot_matlabtmp_*.m" ! -name "robot_matlabtmp_all.m"`; do
  cat $f >> $zd
done
