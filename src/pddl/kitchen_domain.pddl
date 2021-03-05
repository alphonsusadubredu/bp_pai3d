(define (domain COOK)
	(:requirements :strips :typing :adl)
	(:types ingredient drawer)
	(:predicates (robot-at-foodstuff-station)
				 (robot-at-wash-station)
				 (robot-at-stove)
				 (in-tray ?x - ingredient)
				 (clean ?x - ingredient)
				 (holding ?x - ingredient)
				 (handempty)
				 (cooked ?x - ingredient)
				 (in-pile ?x - ingredient)
				 (on-stove ?x - ingredient)
				 (open ?y - drawer)
				 (location ?x ?y)
				 (observed ?x)
				 (served ?x)
				 (in-distribution ?x)
				 (tray-in-hand)
				 (robot-at-mug-station)
				 (food-in-saucepan)
				 (holding-cup)
				 (cup-empty)
				 (cup-at-station)
				 (can-cook)
				 (water-in-saucepan)
	)

	(:action open-drawer
			:parameters (?y - drawer)
			:precondition  (not (open ?y))
			:effect  (open ?y) 
	)

	(:action close-drawer
			:parameters (?y - drawer)
			:precondition (open ?y)
			:effect (not (open ?y))
	)

	(:action inspect
			:parameters (?x - ingredient ?y - drawer)
			:precondition (open ?y)
			:effect (when (location ?x ?y) (observed ?x))
	)

	(:action pick
			:parameters (?x - ingredient)
			:precondition (and (observed ?x) (handempty))
			:effect
			(and (holding ?x)
				 (not (handempty))) 
	)

	(:action put-on-tray
			:parameters (?x - ingredient)
			:precondition (and (holding ?x) (not (in-tray ?x)) (cooked ?x))  
			:effect
			(and (in-tray ?x)
				 (not (in-pile ?x))
				 (not (on-stove ?x))
				 (not (holding ?x))
				 (handempty)
			)
	)

	(:action carry-tray
			:parameters (?x - ingredient)
			:precondition (and (not (tray-in-hand)) (handempty) (in-tray ?x))
			:effect (tray-in-hand)
	)

	(:action wash
			:parameters (?x - ingredient)
			:precondition (and (holding ?x)  (not (clean ?x)) (robot-at-wash-station))
			:effect (and (holding ?x) (clean ?x)) 
	)

	(:action go-to-wash-station
			:parameters ()
			:precondition (and (not (robot-at-wash-station)) )
			:effect 
			(and (robot-at-wash-station)
				 (not (robot-at-foodstuff-station))
				 (not (robot-at-stove))
				 (not (robot-at-mug-station))
			)
	)

	(:action put-food-in-saucepan
			:parameters (?x - ingredient)
			:precondition (and (holding ?x)  (not (cooked ?x)) (robot-at-stove))
			:effect (and (food-in-saucepan) (handempty) (observed ?x))
	)

	(:action go-to-stove
			:parameters ()
			:precondition  ()
			:effect
			(and (robot-at-stove)
				 (not (robot-at-wash-station))
				 (not (robot-at-foodstuff-station))
				 (not (robot-at-mug-station))
			)
	)

	(:action distribute
			:parameters (?x - ingredient)
			:precondition (and (holding ?x) (cooked ?x))
			:effect (in-distribution ?x)
	)

	(:action serve
			:parameters (?x - ingredient)
			:precondition (in-distribution ?x)
			:effect (served ?x)
	)

	(:action go-to-mug-station
			:parameters ()
			:precondition ()
			:effect
			(and (robot-at-mug-station)
				 (not (robot-at-wash-station))
				 (not (robot-at-foodstuff-station))
				 (not (robot-at-stove)))
	)

	(:action get-cup
		:parameters ()
		:precondition (robot-at-mug-station)
		:effect
		(and (holding-cup)
			 (cup-empty)
		)
	)

	(:action fill-with-water
		:parameters ()
		:precondition (and (robot-at-wash-station) (holding-cup) (cup-empty))
		:effect
		(and (holding-cup)
			 (not (cup-empty))
		)
	)

	(:action pour-water-in-saucepan
		:parameters ()
		:precondition (and (robot-at-stove) (holding-cup) (not (cup-empty)))
		:effect
		(and (water-in-saucepan)
			 (cup-empty)
			 (holding-cup)
		)
	)

	(:action put-back-cup
		:parameters ()
		:precondition (and (holding-cup) (water-in-saucepan) (cup-empty) (robot-at-mug-station))
		:effect 
		(and (cup-at-station)
			 (can-cook)
		)
	)

	(:action cook
		:parameters (?x - ingredient)
		:precondition (and (can-cook) (not (cooked ?x)) (water-in-saucepan) (robot-at-stove))
		:effect
		(and (cooked ?x) (observed ?x))
	)
)	