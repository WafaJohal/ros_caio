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


% actions will be ranked in categories
% by specifying that after their execution a given (categorisation) effect is true
% the fact that an agent subject to an emotion wants to achieve that specific effect is
% asserted in an inference rule in pleiad_demo.pl

% TODO October 2012
% see pleiad_demo.pl for tendency handling rules : to be moved here
% and rephrased



% Oatley and Johnson-Laird's action tendencies for their 5 basic emotions
%    - joy: continue plan, modify if needed
%    - sadness: do nothing or find new plan
%    - fear: stop current plan, monitor environment, keep posted or flee
%    - anger: try again, attack
%    - disgust: reject substance, withdraw on oneself 

% attack category (after anger)
:- kassert(bel(I,after(I,insulter(J),attack(J)),1,_,tendency)).
:- kassert(bel(I,after(I,attaquer(J),attack(J)),1,_,tendency)).

% escape category (after fear) 
:- kassert(bel(I,after(I,courir,fuir),1,_,tendency)).

% other fear tendencies : monitor environment, freeze


% Rk: these have a special type of belief (tendency) not used anywhere else
% so that they can be distinguished when searching for action tendencies (eg in planning)


