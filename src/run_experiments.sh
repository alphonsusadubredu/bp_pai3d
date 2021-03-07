#!/bin/bash
for ((k=1; k<=2; k++))
do
	for ((j=1; j<=4; j++))
	do
		for ((i=1; i<=3; i++))
		do 
			python run_bhcsp.py $i $j $k
			sleep 15
		done
	done
done

# sleep 20

# for ((j=1; j<=4; j++))
# do
# 	for ((i=1; i<=3; i++))
# 	do 
# 		python run_bhcsp.py $i $j 3
# 		sleep 15
# 	done
# done
