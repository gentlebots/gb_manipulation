(define (domain robocup)
(:requirements :strips :typing :adl :fluents :durative-actions :typing)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
    robot
    zone
    object
    subzone
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(robot_at ?r - robot ?z - zone)
(object_at ?o - object ?z - zone)

(subzone_at ?sz - subzone ?z - zone)
(free ?sz - subzone)
(object_picked ?r - robot ?o - object ?z - zone)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;


;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action pick
    :parameters (?r - robot ?o - object ?z - zone)
    :duration ( = ?duration 5)
    :condition (and
        (over all(robot_at ?r ?z))
    )
    :effect (and
        (at end(object_picked ?r ?o ?z))
    )
)

(:durative-action place
    :parameters (?r - robot ?o - object ?z - zone ?sz - subzone)
    :duration ( = ?duration 5)
    :condition (and
        (over all(robot_at ?r ?z))
        (over all(subzone_at ?sz ?z))
        (at start(free ?sz))
    )
    :effect (and
        (at end(not(free ?sz)))
        (at end(object_at ?o ?z))
    )
)

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
