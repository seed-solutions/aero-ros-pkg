# grep last roseus
line=$(grep -n 'roseus' $1 | tail -1 | cut -f1 -d:)
tail -n +$line $1 | grep "irteusgl" | cut -f2- -d'$' | sed "{s/ /${2}/}" | sed "s/$/${3}/" > remade_log.txt
sed -i '/send \*ri\* :angle-vector/d' remade_log.txt
sed -i '/()/d' remade_log.txt
