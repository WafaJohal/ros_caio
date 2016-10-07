%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                                                       %%
%% 				PLEIAD Reasoning Engine					 %%
%%                                                       %%
%% Author : Wafa Johal      							 %%
%% Version : v3 - June 2015	                             %%
%% Based on Inaffect & PLEIAD, 							 %%
%%		developed by Carole Adam 2007 					 %%
%%		developed by Jérémy Riviere 2012				 %%
%% Application to the MOCA project						 %%
%%														 %%
%%    ------- DO NOT DISTRIBUTE -------                  %%
%%                                                       %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

:- kassert(current_agent(nao)).

%%%%%%%%%%%%%%%%%%%%%%
%% Domain Knowledge %%
%%%%%%%%%%%%%%%%%%%%%%

%% Context (what the argument was about)
:- kassert(bel(nao,not(fightended(nao,wafa)),0.8,0,conviction)).
% :- kassert(bel(nao,committed(nao,_,fightended(nao,wafa)),1,0,conviction)).

% Domain plans : reconciliate
:- kassert(bel(nao,instant(_T) -> after(nao,action(nao,makeup(nao,User)),fightended(nao,User)),1,0,planification)).
:- kassert(bel(nao,before(nao,action(nao,makeup(nao,User)),believe(nao,forgive(User,nao),1,_T)),1,0,planification)).
:- kassert(bel(nao,instant(T) -> after(User,accept_demand(User,nao,forgive(User,nao)),bel(nao,forgive(User,nao),1,T,_)),1,0,planification)).
:- kassert(bel(nao,instant(T) -> after(User,refuse(User,nao,forgive(User,nao)),bel(nao,not(bestfriend(User,nao)),1,T,_)),1,0,planification)).
:- kassert(bel(nao,instant(T) -> after(User,ask(User,nao,wait(forgive(User,nao))),bel(nao,resp(User,not(forgive(User,nao)),T),1,T,_)),1,0,planification)).


%%%%%%%%%%%%
%% Ideals %%
%%%%%%%%%%%%
:- kassert(ideal(nao,bestfriend(_User,nao),0.7)).

%%%%%%%%%%%
%% Goals %%
%%%%%%%%%%%
:- kassert(goal(nao,saw(nao,_Speaker),1)).
:- kassert(goal(nao,resp(nao,fightended(nao,wafa),_T),0.7)).
:- kassert(goal(nao,resp(wafa,forgive(wafa, nao),_T), 0.7)).


%%%%%%%%%%%%%%%%%%%
%% Acquaintances %%
%%%%%%%%%%%%%%%%%%%
:- kassert(bel(nao,ami(nao,wafa,1),1,0,conviction)). 
:- kassert(trust(nao,_User)).

%%%%%%%%%%%%%%%%%
%% Personality %%
%%%%%%%%%%%%%%%%%
seuil_expression(nao,0.2).
seuil_ressenti(nao,0.4).
