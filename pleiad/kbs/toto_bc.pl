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

% KB of companion agent toto, for child ben
:- kassert(current_agent(toto)).


%%%%%%%%%%%%%%%%%%%%%%
%% Domain Knowledge %%
%%%%%%%%%%%%%%%%%%%%%%

% Devoirs
:- kassert(bel(toto,value(heure,17)->heuredevoirs,1,0,conviction)).

:- kassert(bel(toto,heuredevoirs->oblig(done(ben,fairedevoirs,_)),1,0,conviction)).


% Scherer - intrinsic pleasantness domain linked
:- kassert(intrinsic_p(devoirs,0.2)).
:- kassert(intrinsic_p(jouer,0.8)).


%%%%%%%%%%%%%%%%%%
%% Domain Plans %%
%%%%%%%%%%%%%%%%%%

% utiliser plutot actiondef pour definir les actions de maniere plus simple (par ex a partir d'un fichier xml)
% puis un predicat assert_actions pour asserter en termes de before after (comme assert_language)

%:- kassert(bel(toto,before(toto,action(toto,book_transport),knowval(toto,place,_)),1,0,planification)).
%:- kassert(bel(toto, after(Qqun,action(Qqun,pay_camping),bel(toto,honest(Qqun),1,_T,deduction)),1,0,conviction)).


%%%%%%%%%%%%
%% Ideals %%
%%%%%%%%%%%%
%:- kassert(ideal(toto,bel(toto,not(garbage),1,_T,_Src),0.8)).
:- kassert(ideal(toto,done(ben,fairedevoirs,_),0.6)).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Goals                            %%
%% sources = emotion, social, task  %%
%% cf prefgoal dans pleiad_perso    %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
:- kassert(goal(toto,saw(toto,ben),1,0,social)).
:- kassert(goal(toto,knowval(toto,heure,_T),1,0,task)).

:- kassert(condgoal(toto,heuredevoirs,done(ben,fairedevoirs,_),1,0,task)).


%%%%%%%%%%%%%%%%%%%
%% Acquaintances %%
%%%%%%%%%%%%%%%%%%%
:- kassert(bel(toto,ami(toto,jeremy,1),1,0,conviction)). 
:- kassert(trust(toto,_User)).

%%%%%%%%%%%%%%%%%%
%% Personality  %%
%%%%%%%%%%%%%%%%%%
:- kassert(seuil_expression(toto,0.4)).
:- kassert(seuil_ressenti(toto,0.2)).

