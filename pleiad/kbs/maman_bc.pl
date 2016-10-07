%%%%%%%%%%%%
%% beliefs
%%%%%%%%%%%%
:- kassert(bel(maman,tombe->(not(saitnager)->noyade),0.7,0,conviction)).
:- kassert(bel(maman,tombe->(saitnager->not(noyade)),0.8,0,conviction)).
:- kassert(bel(maman,sauve->not(noyade),0.8,conviction)).


%%%%%%%%%%%%%%%
%% lois d'action
%% comment ecrire qu'elle croit qu'après telle action on a tel effet
%% par ex après que qqun ait sauté les chances de noyade baissent
%% et baissent encore plus si c'est un pompier (loi conditionnelle)


%%%%%%%%%%%
%% idéaux conditionnels
%%%%%%%%%%%

:- kassert(bel(maman,not(blesse)->unideal(maman,passauter,1),1,0,conviction)).
:- kassert(bel(maman,pompier->unideal(pompier,passauter,1),1,0,conviction)).
:- kassert(bel(maman,hblesse->ideal(homme,sauter,0.1),1,0,conviction)).


%%%%%%%%%%%%
%% ideals
%%%%%%%%%%%%
:- kassert(bel(maman,ideal(pompier,sauve,1),1,0,conviction)). 
:- kassert(bel(maman,ideal(passant,sauve,0.77),1,0,conviction)).
:- kassert(bel(maman,ideal(_,sauter,0.8),1,0,conviction)).
:- kassert(bel(maman,unideal(_,passauter,0.7),1,0,conviction)).

%%%%%%%%%%%%%%%%%
%% expectations
%%%%%%%%%%%%%%%%%
:- kassert(bel(maman,unexpected(passant,savelife,0.8),1,0,conviction)). 
:- kassert(bel(maman,unexpected(pompier,savelife,0.1),1,0,conviction)). 
:- kassert(bel(maman,unexpected(maman,passauter,1),1,0,conviction)).
:- kassert(bel(maman,unexpected(homme,sauter,0.8),1,0,conviction)).
:- kassert(bel(maman,unexpected(_,passauter,0.7),1,0,conviction)).


%%%%%%%%%%%%%%%
%% desires
%%%%%%%%%%%%%%%
:- kassert(desire(maman,not(noyade),1)).
:- kassert(desire(maman,beautemps,0.7)).

%%%%%%%%%%%%%%
%% undesires
%%%%%%%%%%%%%%
:- kassert(undesire(maman,noyade,1)).
:- kassert(undesire(maman,not(beautemps),0.8)).

%%%%%%%%%%%
%% focus
%%%%%%%%%%%


%%%%%%%%%%%%%
%% amis
%%%%%%%%%%%%%



%%%%%%%%%%%%%%
%% ennemis
%%%%%%%%%%%%%%



