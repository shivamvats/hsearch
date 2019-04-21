Coding Standards
================

* Method/Function naming policy:
    - camelCase for all methods/functions.
    - exceptions:
        - virtual functions start with a capital letter.

* Public/Private policy:
    Nothing is private. Everything is public.

* Getter and Setters
    - set*() defines setters.
    - there are no setter methods. Directly access the member variables.
    
* Variable naming policy:
    - all variables are in lower case, words separated by underscore.
    - pointers begin with an underscore.

Design
======

* No explicit Graph/Lattice class is required.
    We represent our lattice in a functional form (via Succs function).
    This removes the necessity to store the lattice as the relevant information
    is stored in the planner.
    
    Hence, we only have PlanningSpace classes that take in the state information
    (in whatever form, eg: RobotState) and apply actions from an ActionSpace object 
    that knows how to act on states of that type.
