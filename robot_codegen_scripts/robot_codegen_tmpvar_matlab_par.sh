#!/bin/bash -e
# Erstelle temporäre Variablen mit Code-Schnipseln, die für die Erzeugung von Matlab-Code benötigt wird.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

repo_pfad=$(pwd)/../
tmp_pfad=$repo_pfad/workdir/tmp
source $repo_pfad/robot_codegen_definitions/robot_env_par.sh
source $repo_pfad/codeexport/${robot_leg_name}/tmp/robot_env.sh

# Kinematikparameter (alle als Vektor): MDH-Parameter und Parameter für Zwangsbedingungen
echo "" > $tmp_pfad/robot_matlabtmp_par_KP.m
i=0;
for Kp in $robot_KP; do
  i=$((i+1));          
  echo "$Kp = pkin($i);" >> $tmp_pfad/robot_matlabtmp_par_KP.m
done

# Gravitationsvektor
echo "" > $tmp_pfad/robot_matlabtmp_g.m
for (( i=1; i<=3; i++ ))
do
	echo "g${i} = g(${i});" >> $tmp_pfad/robot_matlabtmp_g.m
done


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
echo "" > $tmp_pfad/robot_matlabtmp_xDP.m
for (( i=1 ; i<=$parallel_NX ; i++ ))
do
	echo "Dx_all(${i}) = xDP(${i});" >> $tmp_pfad/robot_matlabtmp_xDP.m
done

# Plattform-Beschleunigungen
echo "" > $tmp_pfad/robot_matlabtmp_xDDP.m
for (( i=1 ; i<=$parallel_NX ; i++ ))
do
	echo "DDx_all(${i}) = xDDP(${i});" >> $tmp_pfad/robot_matlabtmp_xDDP.m
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

# Alle Code-Schnipsel für Variablendefinitionen in eine Datei schreiben
zd=$tmp_pfad/robot_matlabtmp_all.m
echo "" > $zd
for f in `find $tmp_pfad -name "robot_matlabtmp_*.m" ! -name "robot_matlabtmp_all.m"`; do
  cat $f >> $zd
done
