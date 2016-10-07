
% top goal of being happy
% goal to be happy already in kbs/commonsense.pl
%:- kassert(goal(killua,happy,1,0)).
:- kassert(desire(killua,happy,1)).

% necessary to define predicate abandoned (todo: place it in pleiad)
:- kassert(abandoned(killua,nothing,0)).

% intention to hike should come from this
%:- kassert(intend(killua,done(killua,hike,_),1,0)).
%:- kassert(intend(killua,happy,1,0)).

:- kassert(bel(killua,after(killua,hike,tired),0.6,0,conviction)).

% 2 actions to make him happy: best is to hike, second choice is to play
:- kassert(bel(killua,after(killua,hike,happy),0.77,0,conviction)).
:- kassert(bel(killua,after(killua,playgames,happy),0.6,0,conviction)).

% if raining : risk of getting wet
:- kassert(bel(killua,raining->after(killua,hike,wet),0.9,0,conviction)).

% kassert(bel(killua,raining,1,0,observation)).

%:- kassert(bel(killua,after(killua,sleep,not(tired)),1,0,conviction)).
:- kassert(bel(killua,after(killua,playgames,not(tired)),1,0,conviction)).


% comment desire not to be tired for scenario 2
%:- kassert(undesire(killua,tired,0.7)).

:- kassert(undesire(killua,wet,0.8)).



% scenario
%1 - ?- eemotion(killua,E,P,D,T).
%E = hope,
%P = [hike, happy],
%D = 0.48999999999999994,
%T = 0 ;
%E = distress,
%P = [hike, tired],
%D = 0.9,
%T = 0 ;
%false.

%2- kassert(bel(killua,raining,1,0,observation)).

% 3- eemotion(killua,E,P,D,T).
%E = hope,
%P = [hike, happy],
%D = 0.48999999999999994,
%T = 0 ;
%E = distress,
%P = [hike, tired],
%D = 0.9,
%T = 0 ;
%E = distress,
%P = [hike, wet],
%D = 0.9,
%T = 0.

% 4- cope(killua,Strat).
% renvoie dans Strat la strategie utilisee

% TODO NEXT = envoyer un stimulus "observer pluie" qui a pour effet la croyance qu'il pleut et declenche donc l'émotion de tristesse de se faire mouiller pendant la rando

% suite scenario = application strategie coping
% pas d'action pour empecher d'etre mouille (si, rester chez soi a jouer ou dormir)
% donc incontrolable
% positive reinterpretation : c'est pas si indesirable d'etre mouille
% wishful thinking : peut-etre que je ne serai pas mouille en fait
