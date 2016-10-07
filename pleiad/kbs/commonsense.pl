

% modus ponens for others
% :- kassert(bel(_Agt1,believe(Agt2,Phi->Psi,D1,T)->(believe(Agt2,Phi,D2,T)->believe(Agt2,Psi,D2*D1,T)),1,T,conviction)).

:- kassert(desire(_,heureux,1)).

% a goal to create the goal predicate - very low intensity so that it does not get priority over scenario goals of each agent
:- kassert(goal(_,happy,0.1,0,task)).

:- kassert(phase(_,perception)).


%%
% rule: agents desire that their activities are something they like
:- kassert(bel(_Agt1,like(Agt2,Activity,Deg) -> desire(Agt2,value(activity(Agt2,_Any),Activity),Deg),1,0,conviction)).

% rule: agents undesire that their activities are something they dislike
:- kassert(bel(_Agt1,dislike(Agt2,Activity,Deg) -> undesire(Agt2,value(activity(Agt2,_Any),Activity),Deg),1,0,conviction)).

% something (un)desirable also is as a future prospect
% (to enable hope/fear about future prospects)
%:- kassert(bel(_Agt1,desire(Agt2,Prop,Deg)->desire(Agt2,future(Prop),Deg),1,0,conviction)).
%:- kassert(bel(_Agt1,undesire(Agt2,Prop,Deg)->undesire(Agt2,future(Prop),Deg),1,0,conviction)).

% infinite loop future(future(future(...)))