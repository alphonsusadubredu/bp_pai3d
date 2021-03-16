(define (problem ROBOT-COOK) 
(:domain COOK) 
 (:objects pear meat_can - ingredient
top-left top-right middle - drawer)
(:init  (robot-at-foodstuff-station) (handempty)  (location pear middle) ) 
(:goal (and  (holding pear)))) 
