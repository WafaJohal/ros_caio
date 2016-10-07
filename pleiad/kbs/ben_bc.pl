%%%%%%%%%%%%
%% beliefs
%%%%%%%%%%%%

% utiliser A->B pour les implications

:- kassert(bel(ben,gocool,0.9,0,conviction)). 
:- kassert(bel(ben,pluie,0.7,0,conviction)). 
:- kassert(bel(ben,chaton,1,0,conviction)). 

%% image kao
:- kassert(bel(ben,desire(kao,chaton,1),1,0,conviction)). 
:- kassert(bel(ben,believe(kao,not(chaton),1,1),1,0,conviction)).
:- kassert(bel(ben,believe(kao,chaton,1,2),1,0,conviction)). 




%%%%%%%%%%%%
%% ideals
%%%%%%%%%%%%
:- kassert(bel(ben,ideal(savelife,_,1),1,0,conviction)). 
:- kassert(bel(ben,unideal(_,kill,1),1,0,conviction)). 
:- kassert(bel(ben,unideal(_,break,1),1,0,conviction)).


%%%%%%%%%%%%%%%%%
%% expectations 
%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%
%% desires
%%%%%%%%%%%%%%%
:- kassert(desire(ben,gocool,0.5)). 
:- kassert(desire(ben,soleil,0.6)).


%%%%%%%%%%%%%%
%% undesires
%%%%%%%%%%%%%%
:- kassert(undesire(ben,pluie,0.6)). 

%%%%%%%%%%%
%% focus
%%%%%%%%%%%
:- kassert(focus(ben,gocool,0.9,0)).
:- kassert(focus(ben,chaton,1,0)).

%%%%%%%%%%%
%% amis
%%%%%%%%%%%
:- kassert(ami(ben,kao,1)).



%%%%%%%%%%%%%
%% ennemis
%%%%%%%%%%%%%
:- kassert(ennemi(ben,koai,1)).


