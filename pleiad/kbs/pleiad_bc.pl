%%%%%%%%%%%%
%% beliefs
%%%%%%%%%%%%
:- kassert(bel(pleiad,questions,0.5,0,conviction)).


%%%%%%%%%%%%
%% ideals
%%%%%%%%%%%%
:- kassert(bel(pleiad,ideal(public,questions,1),1,0,conviction)).


%%%%%%%%%%%%%%%%%
%% expectations
%%%%%%%%%%%%%%%%%
:- kassert(bel(pleiad,unexpected(public,questions,1),1,0,conviction)).

%%%%%%%%%%%%%%%
%% desires
%%%%%%%%%%%%%%%
:- kassert(desire(pleiad,questions,1)).


%%%%%%%%%%%%%%
%% undesires
%%%%%%%%%%%%%%


%%%%%%%%%%%
%% focus
%%%%%%%%%%%


%%%%%%%%%%%%%
%% amis
%%%%%%%%%%%%%



%%%%%%%%%%%%%%
%% ennemis
%%%%%%%%%%%%%%



