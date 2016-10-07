%%%%%%%%%%%%
%% beliefs
%%%%%%%%%%%%
:- kassert(bel(clem,ordicasse,1,0,observation)).

:- kassert(bel(clem,ordisolide->not(ordicasse),1,0,conviction)).
:- kassert(bel(clem,ordinouveau->not(ordicasse),1,0,conviction)).


:- kassert(bel(clem,action(ben,voler_lunettes,0),1,0,observation)).
:- kassert(bel(clem,resp(ben,voler_lunettes,lunettes_disparues),1,0,observation)).
:- kassert(bel(clem,lunettes_disparues,1,0,observation)).

:- kassert(bel(clem,retardcine,0.97,0,observation)).
:- kassert(bel(clem,action(ben,jouerds,0),0.87,0,observation)).
:- kassert(bel(clem,doitallercine,1,0,communication)).
:- kassert(bel(clem,jouelongtemps->resp(ben,jouerds,retardcine),0.81,0,conviction)).

%%%%%%%%%%%%
%% ideals
%%%%%%%%%%%%
:- kassert(bel(clem,unideal(ben,voler_lunettes,1),1,0,conviction)).

% ideaux conditionnels (prevoir l'annulation de l'ideal qd la condition devient fausse
:- kassert(bel(clem,doitallercine->unideal(_,jouerds,0.57),1,0,conviction)).
:- kassert(bel(clem,not(doitallercine)->not(unideal(_,jouerds,0.57)),1,0,conviction)).
% attention si plusieurs conditions, toutes doivent etre fausses pour annuler l'idéal

%%%%%%%%%%%%%%%%%
%% expectations
%%%%%%%%%%%%%%%%%
:- kassert(bel(clem,unexpected(ben,voler_lunettes,1),1,0,conviction)).

% conditionnels (prevoir l'annulation)
:- kassert(bel(clem,doitallercine->unexpected(_,jouerds,0.57),1,0,conviction)).
:- kassert(bel(clem,not(doitallercine)->not(unexpected(_,jouerds,0.57)),1,0,conviction)).


%%%%%%%%%%%%%%%
%% desires
%%%%%%%%%%%%%%%
:- kassert(desire(clem,not(ordicasse),1)).

%%%%%%%%%%%%%%
%% undesires
%%%%%%%%%%%%%%
:- kassert(undesire(clem,lunettes_disparues,1)).

:- kassert(undesire(clem,retardcine,0.87)).

%%%%%%%%%%%
%% focus
%%%%%%%%%%%

:- kassert(focus(clem,ordicasse,1,0)).
:- kassert(focus(clem,voler_lunettes,1,0)).
:- kassert(focus(clem,lunettes_disparues,1,0)).

:- kassert(focus(clem,retardcine,1,0)).

%%%%%%%%%%%%%
%% amis
%%%%%%%%%%%%%

:- kassert(ami(clem,kao,1)).



%%%%%%%%%%%%%%
%% ennemis
%%%%%%%%%%%%%%

:- kassert(ennemi(clem,lulu,1)).


%%%%%%%%%%%%%%
%% actions 
%%%%%%%%%%%%%%

:- kassert(bel(clem,after(clem,courir,heureux),1,0,conviction)).

:- kassert(bel(clem,after(clem,reparerordi,not(ordicasse)),1,0,conviction)).


%%%%%%%%%%%%%%%%%%%%
%% prefs de coping
%%%%%%%%%%%%%%%%%%%%

:- kassert(prefcop(clem,
[active_coping(_),seek_material_support(_),denial(_),
positive_reinterpretation(_),mental_disengagement(_),shift_responsability(_)])).
