%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                                                       %%
%% PLEIAD : ProLog Emotional Intelligent Agents Designer %%
%%                                                       %%
%% Author : Carole Adam                                  %%
%% Version : v7 - October 2007                           %%
%%                                                       %%
%%    ------- DO NOT DISTRIBUTE -------                  %%
%%                                                       %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% ideals and expectations known by all agents
%% reloaded when any KB is reloaded

% Rk October 2012
% these are domain dependent and should be in a domain kb in the kb folder

:- kassert(ideal(_,savelife,1)).

:- kassert(unideal(_,kill,1)).
:- kassert(unideal(_,break,0.5)).

:- kassert(unexpected(_,break,0.7)).

:- kassert(bel(_,ideal(_,savelife,1),1,0,observation)).
:- kassert(bel(_,unideal(_,kill,1),1,0,observation)).
:- kassert(bel(_,unideal(_,break,0.5),1,0,observation)).

:- kassert(bel(_,unexpected(_,break,0.7),1,0,observation)).
