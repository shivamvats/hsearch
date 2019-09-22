To Do
=====

* [ ] Visualization for Graph Search:

        * Visualize actions
        * Visalize nodes
        * Make it interactive with hierarchial details
* [x] Add CollisionChecker

Coding Standards
================

* Method/Function naming policy:
    - camelCase for all methods/functions.
    - exceptions:
        - virtual functions start with a capital letter.
        
* Private/Public Policy:
    - Keep things public as much as possible.
    - However, for inheritance to work correctly, some things might need to be kept private.
    - Such member objects will require setters/getters.
    
* Variable naming policy:
    - all variables are in lower case, words separated by underscore.
    - pointers end with `_ptr`.

Design
======

* No explicit Graph/Lattice class is required.
    We represent our lattice in a functional form (via Succs function).
    This removes the necessity to store the lattice as the relevant information
    is stored in the planner.
    
    Hence, we only have PlanningSpace classes that take in the state information
    (in whatever form, eg: RobotState) and apply actions from an ActionSpace object 
    that knows how to act on states of that type.
