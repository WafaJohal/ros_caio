%%%%%%%%%%%%
%% beliefs
%%%%%%%%%%%%

:- kassert(bel(employee,boncv->entretien,0.8,0,conviction)). 
:- kassert(bel(employee,not(boncv)->not(entretien),0.95,0,conviction)).
:- kassert(bel(employee,bonentretien->emploi,0.6,0,conviction)).
:- kassert(bel(employee,not(entretien)->not(emploi),1,0,conviction)).
:- kassert(bel(employee,entretien->emploi,0.2,0,conviction)).

% options bel(employee,boncv,1,0)
% option2 bel(employee,entretien->bonentretien,0.7,0).
%      ou bel(employee,entretien->entretienrate,0.7,0).

%% image kao



%%%%%%%%%%%%
%% ideals
%%%%%%%%%%%%
%:- kassert(bel(employee,ideal(savelife,_,1),1,0)). 
%:- kassert(bel(ben,unideal(_,kill,1),1,0)). 
%:- kassert(bel(ben,unideal(_,break,1),1,0)).


%%%%%%%%%%%%%%%%%
%% expectations 
%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%
%% desires
%%%%%%%%%%%%%%%
:- kassert(desire(employee,entretien,0.5)). 
% regle de derivation / planification de desirs:
% desire un emploi, il faut un entretien pour avoir un emploi, donc desire % entretien ?
:- kassert(desire(employee,bonentretien,0.6)).
:- kassert(desire(employee,emploi,0.6)).
:- kassert(desire(employee,boncv,0.6)).


%%%%%%%%%%%%%%
%% undesires
%%%%%%%%%%%%%%
:- kassert(undesire(employee,not(entretien),0.5)).
:- kassert(undesire(employee,not(bonentretien),0.6)). 
:- kassert(undesire(employee,not(emploi),0.9)). 
:- kassert(undesire(employee,not(boncv),0.6)).


%%%%%%%%%%%
%% focus
%%%%%%%%%%%
:- kassert(focus(employee,emploi,0.9,1)).
:- kassert(focus(employee,entretien,0.8,0)).
:- kassert(focus(employee,boncv,0.3,0)).
:- kassert(focus(employee,bonentretien,1,0)).
% il faudrait propager automatiquement le focus de entretien à bonentretien

%%%%%%%%%%%
%% amis
%%%%%%%%%%%
%:- kassert(ami(ben,kao,1)).



%%%%%%%%%%%%%
%% ennemis
%%%%%%%%%%%%%
:- kassert(ennemi(ben,koai,1)).


