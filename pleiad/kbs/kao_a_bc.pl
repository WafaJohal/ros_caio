%%%%%%%%%%%%
%% beliefs
%%%%%%%%%%%%

:- kassert(bel(kao,winloto,0.5,0)). 
:- kassert(bel(kao,winloto,1,2)).

%%%%%%%%%%%%
%% ideals
%%%%%%%%%%%%
:- kassert(bel(kao,ideal(_,savelife,1),1,0)). 
:- kassert(bel(kao,unideal(_,kill,1),1,0)). 
:- kassert(bel(kao,unideal(_,break,1),1,0)).


%%%%%%%%%%%%%%%%%
%% expectations
%%%%%%%%%%%%%%%%%
:- kassert(bel(kao,unexpected(kao,savelife,0.8),1,0)). 
:- kassert(bel(kao,unexpected(ben,break,0.9),1,0)).

%%%%%%%%%%%%%%%
%% desires
%%%%%%%%%%%%%%%

:- kassert(desire(kao,winloto,1)).

%%%%%%%%%%%%%%
%% undesires
%%%%%%%%%%%%%%


%%%%%%%%%%%
%% focus
%%%%%%%%%%%


%%%%%%%%%%%%%
%% amis
%%%%%%%%%%%%%
:- kassert(ami(kao,ben,1)).


%%%%%%%%%%%%%%
%% ennemis
%%%%%%%%%%%%%%
:- kassert(ennemi(kao,lolo,1)). 


