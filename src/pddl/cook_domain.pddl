(define (domain COOK)
	(:requirements :strips :typing :adl)
	(:types ingredient)
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
				 (tray-in-hand)
	)

	(:action pick
			:parameters (?x - ingredient)
			:precondition (handempty)
			:effect
			(and (holding ?x)
				 (not (handempty))) 
	)
	(:action put-on-tray
			:parameters (?x - ingredient)
			:precondition (and (holding ?x) (not (in-tray ?x)))  
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
			:precondition (and (holding ?x) (in-tray ?x) (not (clean ?x)) (robot-at-wash-station))
			:effect (clean ?x) 
	)

	(:action go-to-wash-station
			:parameters ()
			:precondition (and (handempty) (not (robot-at-wash-station)) (tray-in-hand))
			:effect 
			(and (robot-at-wash-station)
				 (not (robot-at-foodstuff-station))
				 (not (robot-at-stove))
			)
	)

	(:action cook
			:parameters (?x - ingredient)
			:precondition (and (holding ?x) (in-tray ?x) (not (cooked ?x)) (robot-at-stove))
			:effect (cooked ?x)
	)

	(:action go-to-stove
			:parameters ()
			:precondition (and (handempty) (not (robot-at-stove)) (tray-in-hand))
			:effect
			(and (robot-at-stove)
				 (not (robot-at-wash-station))
				 (not (robot-at-foodstuff-station))
			)
	)
)	