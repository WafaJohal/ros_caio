
% GROS PB TEMPORELS !!!
% comment exprimer qu'il croit que les pompiers VONT venir
% et que si les pompiers viennent A TEL INSTANT alors ils eteindront le feu
% etc ...

%%%%%%%%%%%%
%% beliefs
%%%%%%%%%%%%

% utiliser A->B pour les implications
%:- kassert(bel(ben,gocool,0.9,0,conviction)).

:- kassert(bel(manager1,hotelIsBurning,1,0,conviction)).

:-
kassert(bel(manager1,hotelIsBurning->hotelWillBeDestroyed,0.8,0,conviction)).

:-
kassert(bel(manager1,not(hotelIsBurning)->not(hotelWillBeDestroyed),0.7,0,conviction)).

:-
kassert(bel(manager1,firemenHere->not(hotelIsBurning),0.7,0,conviction)).

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

:- kassert(undesire(manager1,hotelWillBeDestroyed,0.85)).


%%%%%%%%%%%%%
%% intentions
%%%%%%%%%%%%%
%:- kassert(intend(planner,position(planner,piscine),1,0)).

%%%%%%%%%%%
%% focus
%%%%%%%%%%%
:- kassert(focus(manager1,hotelIsBurning,0.9,0)).

:- kassert(focus(manager1,hotelWillBeDestroyed,0.9,0)).

%%%%%%%%%%%
%% amis et ennemis
%%%%%%%%%%%
%:- kassert(ami(ben,kao,1)).
%:- kassert(ennemi(ben,koai,1)).

%%%%%%%%%%%%%%
%% actions
%%%%%%%%%%%%%%
%
% il faudrait des after (et before ?) avec des probas
% pour avoir plusieurs résultats envisageables d'une meme action
% for now les degrés de proba sont dans le degré de croyance...

:-
kassert(bel(manager1,after(pompiers,extinguish,not(hotelIsBurning)),1,0,conviction)).

%:-
%kassert(bel(manager1,after(pompiers,extinguish,not(hotelWillBeDestroyed)),0.8,0,conviction)).

:- kassert(bel(manager1,after(manager1,request(manager1,
pompiers,done(pompiers,extinguish,_),_),firemenComing),1,0,conviction)).

:- kassert(bel(manager1,before(manager1,request(manager1,
pompiers,done(pompiers,extinguish,_),_),avoirTelephone),1,0,conviction)).

:-
kassert(bel(manager1,after(manager1,trouverTelephone,avoirTelephone),0.9,0,conviction)).

:-
kassert(bel(manager1,after(manager1,courirChercherTelephone,avoirTelephone),0.9,0,conviction)).

:-
kassert(bel(manager1,after(manager1,courirChercherTelephone,fuir),0.9,0,tendency)).

:-
kassert(bel(manager1,before(manager1,attendrePompiers,firemenComing),1,0,conviction)).

:-
kassert(bel(manager1,after(manager1,attendrePompiers,firemenHere),1,0,conviction)).














%
