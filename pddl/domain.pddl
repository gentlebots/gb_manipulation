(define (domain cooking)
(:requirements :strips :typing :adl :fluents :durative-actions :typing)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
object
zone 
robot
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(battery_full ?r - robot)
(robot_at ?r - robot ?z - zone)
(object_at ?o - object ?z - zone)

(is_delivery_zone ?z - zone)
(is_food_zone ?z - zone)

(object_picked ?r - robot ?o - object)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;


;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?z1 ?z2 - zone)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?z1)))
    :effect (and
        (at start(not(robot_at ?r ?z1)))
        (at end(robot_at ?r ?z2))
    )
)

(:durative-action pick
    :parameters (?r - robot ?o - object)
    :duration ( = ?duration 5)
    :condition (and
    )
    :effect (and
        (at end(object_picked ?r ?o))
    )
)

(:durative-action place
    :parameters (?r - robot ?o - object ?z1 ?z2 - zone)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?z1))
        (at start(object_at ?o ?z1))
    )
    :effect (and
        (at start(not(robot_at ?r ?z1)))
        (at end(robot_at ?r ?z2))
        (at start(not(object_at ?o ?z1)))
        (at end(object_at ?o ?z2))
    )
)

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
