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

% created October 2012



%%%%%%%%%%%%%%%%%
% RELATIONSHIPS %
%%%%%%%%%%%%%%%%%

vecteur_amis(A,L) :- findall(ami(A,B,Deg),ami(A,B,Deg),L1),
				findall(ennemi(A,B,Deg),ennemi(A,B,Deg),L2),
				merge(L1,L2,L).



%%%%%%%%%%%
%% TRUST %%
%%%%%%%%%%%

% TODO October 2012
% manage trust with other agents
% influence on beliefs and everything

% predicate trust(Agt1,Agt2,Degree)
% used in some inference rules for belief adoption (cf pleiad_demo.pl)
% also updated after discovered lies, or wrong actions, etc
