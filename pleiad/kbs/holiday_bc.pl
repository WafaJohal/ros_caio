%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                                                       %%
%% PLEIAD Reasoning Engine	                	 %%
%%                                                       %%
%% Author : Jeremy Riviere, Carole Adam                  %%
%% Version : v2 - February 2012                          %%
%% Based on PLEIAD, developed by Carole Adam 2007        %%
%% Application to the CECIL project			 %%
%%							 %%
%%    ------- DO NOT DISTRIBUTE -------                  %%
%%                                                       %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

:- kassert(current_agent(agent)).


%%%%%%%%%%
%% TEST %%
%%%%%%%%%%

:- kassert(bel(agent,instant(_T) -> after(agent,action(agent,testaction1),youpi),1,0,planification)).
:- kassert(bel(agent,before(agent,action(agent,testaction1),believe(agent,testcond,1.0,_T)),1,0,planification)).
:- kassert(bel(agent,instant(T) -> after(agent,action(agent,testaction2),bel(agent,testcond,1.0,T,deduction)),1,0,planification)).

% :- kassert(bel(agent,instant(T) -> after(agent,action(agent,testaction2),testcond),1,0,planification)).


%%%%%%%%%%%%%%%%%%%%%%
%% Domain Knowledge %%
%%%%%%%%%%%%%%%%%%%%%%

% Campings
:- kassert(bel(agent,camping(forest_camping),1,0,conviction)).
:- kassert(bel(agent,camping(mountain_camping),1,0,conviction)).

:- kassert(bel(agent,good(forest_camping),1,0,conviction)).
:- kassert(bel(agent,good(mountain_camping),0.6,0,conviction)).

% Hiking
:- kassert(bel(agent,hiking(crest_hiking),1,0,conviction)).
:- kassert(bel(agent,hiking(lake_hiking),1,0,conviction)).
:- kassert(bel(agent,hiking(forest_hiking),1,0,conviction)).

:- kassert(bel(agent,alt_level(crest_hiking,1500),1,0,conviction)).
:- kassert(bel(agent,alt_level(lake_hiking,1000),1,0,conviction)).
:- kassert(bel(agent,alt_level(forest_hiking,300),1,0,conviction)).

:- kassert(bel(agent,not(kids(crest_hiking)),1,0,conviction)).
:- kassert(bel(agent,not(kids(lake_hiking)),1,0,conviction)).
:- kassert(bel(agent,kids(forest_hiking),1,0,conviction)).

:- kassert(bel(agent,landscape(crest_hiking),1,0,conviction)).
:- kassert(bel(agent,landscape(lake_hiking),1,0,conviction)).
:- kassert(bel(agent,not(landscape(forest_hiking)),1,0,conviction)).

:- kassert(bel(agent,knowval(jeremy,date,_),1,0,conviction)).
:- kassert(bel(agent,knowval(jeremy,place,_),1,0,conviction)).

% Scherer - intrinsic pleasantness domain linked
:- kassert(intrinsic_p(organised,0.7)).
:- kassert(intrinsic_p(vacations,0.8)).
:- kassert(intrinsic_p(holiday,0.8)).
:- kassert(intrinsic_p(landscape,0.8)).
:- kassert(intrinsic_p(camping,0.8)).
:- kassert(intrinsic_p(hiking,0.8)).
:- kassert(intrinsic_p(pay_camping,0.2)).
:- kassert(intrinsic_p(collect_garbage,0.8)).
:- kassert(intrinsic_p(honest,1)).

% Domain plans : organise holidays
:- kassert(bel(agent,instant(_T) -> after(agent,action(agent,organise),organised),1,0,planification)).

:- kassert(bel(agent,before(agent,action(agent,organise),believe(agent,transport_booked,1.0,_T)),1,0,planification)).
:- kassert(bel(agent,before(agent,action(agent,organise),believe(agent,accomodation_booked,1.0,_T)),1,0,planification)).
:- kassert(bel(agent,instant(T) -> after(agent,action(agent,book_transport),bel(agent,transport_booked,1.0,T,deduction)),1,0,planification)).
:- kassert(bel(agent,instant(T) -> after(agent,action(agent,book_accomodation),bel(agent,accomodation_booked,1.0,T,deduction)),1,0,planification)).
:- kassert(bel(agent,before(agent,action(agent,book_accomodation),believe(agent,believe(jeremy,goal(agent,resp(agent,organised,_T),_D),_D2,_T1),1.0,_T2)),1,0,planification)).
:- kassert(bel(agent,before(agent,action(agent,book_transport),knowval(agent,place,_)),1,0,planification)).
:- kassert(bel(agent,before(agent,action(agent,book_transport),knowval(agent,date,_)),1,0,planification)).

% Domain actions
:- kassert(bel(agent, after(Qqun,action(Qqun,pay_camping),bel(agent,honest(Qqun),1,_T,deduction)),1,0,conviction)).
:- kassert(bel(agent, after(Qqun,action(Qqun,unpay_camping),bel(agent,not(honest(Qqun)),1,_T,deduction)),1,0,conviction)).

:- kassert(bel(agent, after(Qqun,action(Qqun,collect_garbage),bel(agent,not(garbage),1,_T,deduction)),1,0,conviction)).


%%%%%%%%%%%%
%% Ideals %%
%%%%%%%%%%%%
:- kassert(ideal(agent,bel(agent,not(garbage),1,_T,_Src),0.8)).
:- kassert(ideal(agent,bel(agent,honest(_Qqun),1,_T,_Src),0.6)).

%%%%%%%%%%%
%% Goals %%
%%%%%%%%%%%
:- kassert(goal(agent,saw(agent,_Speaker),1)).


%%%%%%%%%%%%%%%%%%%
%% Acquaintances %%
%%%%%%%%%%%%%%%%%%%
:- kassert(bel(agent,ami(agent,jeremy,1),1,0,conviction)). 
:- kassert(trust(agent,_User)).

%%%%%%%%%%%%%%%%%%
%% Personality  %%
%%%%%%%%%%%%%%%%%%
:- kassert(seuil_expression(agent,0.2)).
:- kassert(seuil_ressenti(agent,0.4)).