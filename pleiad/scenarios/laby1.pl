:- kassert(position(0, 0, wall)).
:- kassert(position(0, 1, entrance)).
:- kassert(position(0, 2, wall)).
:- kassert(position(0, 3, wall)).
:- kassert(position(0, 4, wall)).
:- kassert(position(1, 0, wall)).
:- kassert(position(1, 1, empty)).
:- kassert(position(1, 2, wall)).
:- kassert(position(1, 3, empty)).
:- kassert(position(1, 4, wall)).
:- kassert(position(2, 0, wall)).
:- kassert(position(2, 1, empty)).
:- kassert(position(2, 2, wall)).
:- kassert(position(2, 3, empty)).
:- kassert(position(2, 4, wall)).
:- kassert(position(3, 0, wall)).
:- kassert(position(3, 1, empty)).
:- kassert(position(3, 2, empty)).
:- kassert(position(3, 3, empty)).
:- kassert(position(3, 4, wall)).
:- kassert(position(4, 0, wall)).
:- kassert(position(4, 1, wall)).
:- kassert(position(4, 2, wall)).
:- kassert(position(4, 3, exit)).
:- kassert(position(4, 4, wall)).
:- kassert(solution([[0,1], [1,1], [2,1], [3,1], [3,2], [3,3], [4,3]])).