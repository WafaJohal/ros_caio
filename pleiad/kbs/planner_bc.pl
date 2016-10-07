%%%%%%%%%%%%
%% beliefs
%%%%%%%%%%%%

% utiliser A->B pour les implications
%:- kassert(bel(ben,gocool,0.9,0,conviction)). 

:- kassert(bel(planner,position(planner,maison),1,0,conviction)).

%% image kao
%:- kassert(bel(ben,desire(kao,chaton,1),1,0,conviction)). 

%%%%%%%%%%%%
%% ideals
%%%%%%%%%%%%
%:- kassert(bel(ben,ideal(savelife,_,1),1,0,conviction)). 

%%%%%%%%%%%%%%%%%
%% expectations 
%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%
%% desires and undesires
%%%%%%%%%%%%%%%
%:- kassert(desire(ben,gocool,0.5)). 
%:- kassert(undesire(ben,pluie,0.6)). 

%%%%%%%%%%%%%
%% intentions
%%%%%%%%%%%%%
:- kassert(intend(planner,position(planner,piscine),1,0)).

%%%%%%%%%%%
%% focus
%%%%%%%%%%%
%:- kassert(focus(ben,gocool,0.9,0)).

%%%%%%%%%%%
%% amis et ennemis
%%%%%%%%%%%
%:- kassert(ami(ben,kao,1)).
%:- kassert(ennemi(ben,koai,1)).

%%%%%%%%%%%%%%
%% actions 
%%%%%%%%%%%%%%

:- kassert(bel(planner,action(aller(maison,piscine)),1,0,conviction)).
:- kassert(bel(planner,action(aller(_,_)),1,0,conviction)).
:- kassert(bel(planner,after(planner,aller(_,Y),position(planner,Y)),1,0,conviction)).
:- kassert(bel(planner,after(planner,aller(X,_),not(position(planner,X))),1,0,conviction)).
:- kassert(bel(planner,before(planner,aller(X,_),position(planner,X)),1,0,conviction)).
:- kassert(bel(planner,before(planner,aller(_,Y),not(position(planner,Y))),1,0,conviction)).

% il faudrait des after (et before ?) avec des probas
% pour avoir plusieurs résultats envisageables d'une meme action


