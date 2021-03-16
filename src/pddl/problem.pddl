(define (problem ROBOT-COOK) 
(:domain COOK) 
 (:objects pear meat_can - ingredient
top-left top-right middle - drawer)
(:init  (clean pear) (holding pear) ) 
(:goal (and  (cooked pear) (observed pear) (handempty)))) 
