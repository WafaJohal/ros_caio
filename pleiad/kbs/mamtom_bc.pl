
%% mutual beliefs
:- kassert(bel(mamtom,not(rangee),1,0,observation)).
:- kassert(bel(mamtom,believe(tom,not(rangee),1,0),1,0,conviction)).
:- kassert(bel(mamtom,believe(tom, believe(mamtom,not(rangee),1,0),1,0),1,0,conviction)).