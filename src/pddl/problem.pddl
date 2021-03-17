(define (problem ROBOT-COOK) 
(:domain COOK) 
 (:objects pear meat_can - ingredient
top-left top-right middle - drawer)
(:init  (cooked pear) (observed pear) (handempty) ) 
(:goal (and  (served pear)))) 
