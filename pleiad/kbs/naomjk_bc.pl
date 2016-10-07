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

% KB of companion agent naomjk, for informing visitors about MAGMA
:- kassert(current_agent(naomjk)).
:- kassert(human(julie)).
:- kassert(human(sylvie)).
:- kassert(human(carole)).



%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Physical properties  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%

% positions
:- kassert(bel(naomjk,position(naomjk,mjk),1,0,conviction)).
:- kassert(bel(naomjk,position(sylvie,homesylvie),1,0,conviction)).
:- kassert(bel(naomjk,position(julie,homejulie),1,0,conviction)).
:- kassert(bel(naomjk,position(kao,mjk),1,0,conviction)).
:- kassert(bel(naomjk,position(wafa,mjk),1,0,conviction)).
:- kassert(bel(naomjk,position(yves,denmark),1,0,conviction)).
:- kassert(bel(naomjk,position(damien,homedamien),1,0,conviction)).

% nao properties
:- kassert(enforme(naomjk)).
:- kassert(possedeticket(nao)).

%%%%%%%%%%%%%%%%%%%%%%
%% Domain Knowledge %%
%%%%%%%%%%%%%%%%%%%%%%

% observed proposition that nao wants to tell everybody
:- kassert(bel(naomjk,news,1,0,observation)).


% other countries
:- kassert(bel(naomjk,othercountry(denmark),1,0,conviction)).
:- kassert(bel(naomjk,othercountry(norway),1,0,conviction)).

% holidays
:- kassert(bel(naomjk,holidays(sylvie),1,0,conviction)).
:- kassert(bel(naomjk,holidays(damien),1,0,conviction)).
:- kassert(bel(naomjk,holidays(yves),1,0,conviction)).

:- kassert(bel(naomjk,holidays(Agt)->not(reademail(Agt)),1,0,conviction)).

% Moves % not(othercountry(Finish))->  OU othercountry(Finish)-> for plane
:- kassert(bel(naomjk,position(Agent,Start)->after(Agent,action(Agent,velo(Agent,Start,Finish)),position(Agent,Finish)),1,0,conviction)).
:- kassert(bel(naomjk,position(Agent,Start)->after(Agent,action(Agent,marche(Agent,Start,Finish)),position(Agent,Finish)),1,0,conviction)).
:- kassert(bel(naomjk,position(Agent,Start)->after(Agent,action(Agent,tram(Agent,Start,Finish)),position(Agent,Finish)),1,0,conviction)).
:- kassert(bel(naomjk,position(Agent,Start)->after(Agent,action(Agent,avion(Agent,Start,Finish)),position(Agent,Finish)),1,0,conviction)).

:- kassert(bel(naomjk,before(Agt,action(Agt,velo(Agt,_Start,Finish)),not(othercountry(Finish))),1,0,conviction)).
:- kassert(bel(naomjk,before(Agt,action(Agt,marche(Agt,_Start,Finish)),not(othercountry(Finish))),1,0,conviction)).
:- kassert(bel(naomjk,before(Agt,action(Agt,tram(Agt,_Start,Finish)),not(othercountry(Finish))),1,0,conviction)).
:- kassert(bel(naomjk,before(Agt,action(Agt,velo(Agt,Start,_Finish)),not(othercountry(Start))),1,0,conviction)).
:- kassert(bel(naomjk,before(Agt,action(Agt,marche(Agt,Start,_Finish)),not(othercountry(Start))),1,0,conviction)).
:- kassert(bel(naomjk,before(Agt,action(Agt,tram(Agt,Start,_Finish)),not(othercountry(Start))),1,0,conviction)).

:- kassert(bel(naomjk,before(Agt,action(Agt,avion(Agt,_Start,Finish)),othercountry(Finish)),1,0,conviction)).


% Positions
:- kassert(bel(naomjk,position(kao,maison),1,0,conviction)).
:- kassert(bel(naomjk,position(wafa,mjk),1,0,conviction)).
:- kassert(bel(naomjk,position(julie,norway),1,0,conviction)).
:- kassert(bel(naomjk,position(yves,denmark),1,0,conviction)).

%:- kassert(bel(toto,heuredevoirs->oblig(done(ben,fairedevoirs,_)),1,0,conviction)).




%%%%%%%%%%%%%%%%%%
%% Domain Plans %%
%%%%%%%%%%%%%%%%%%

% utiliser plutot actiondef pour definir les actions de maniere plus simple (par ex a partir d'un fichier xml)
% puis un predicat assert_actions pour asserter en termes de before after (comme assert_language)

:- kassert(bel(naomjk,before(Agent,action(Agent,marche(Agent,PosA,_PosB)),position(Agent,PosA)),1,0,conviction)).
:- kassert(bel(naomjk,before(Agent,action(Agent,velo(Agent,PosA,_PosB)),position(Agent,PosA)),1,0,conviction)).
:- kassert(bel(naomjk,before(Agent,action(Agent,tram(Agent,PosA,_PosB)),position(Agent,PosA)),1,0,conviction)).

:- kassert(bel(naomjk,before(Agent,action(Agent,tram(Agent,_PosA,_PosB)),possedeticket(Agent)),1,0,conviction)).
:- kassert(bel(naomjk,before(Agent,action(Agent,velo(Agent,_PosA,_PosB)),enforme(Agent)),1,0,conviction)).

:- kassert(bel(naomjk,after(Agent,action(Agent,mangeptidej),enforme(Agent)),1,0,conviction)).

:- kassert(bel(naomjk,after(Agent,action(Agent,acheteticket),possedeticket(Agent)),1,0,conviction)).

:- kassert(bel(naomjk,position(Hearer,Pos)->before(naomjk,speechact(inform,naomjk,Hearer,_Prop),position(naomjk,Pos)),1,0,conviction)).

   

%:- kassert(bel(toto,before(toto,action(toto,book_transport),knowval(toto,place,_)),1,0,planification)).
%:- kassert(bel(toto, after(Qqun,action(Qqun,pay_camping),bel(toto,honest(Qqun),1,_T,deduction)),1,0,conviction)).


%%%%%%%%%%%%
%% Ideals %%
%%%%%%%%%%%%
%:- kassert(ideal(naomjk,bel(toto,not(garbage),1,_T,_Src),0.8)).
%:- kassert(ideal(naomjk,done(ben,fairedevoirs,_),0.6)).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Goals                            %%
%% sources = emotion, social, task  %%
%% cf prefgoal dans pleiad_perso    %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

:- kassert(goal(naomjk,believe(sylvie,news,_,_),1,0,task)).
:- kassert(goal(naomjk,believe(julie,news,_,_),1,0,task)).

%:- kassert(condgoal(toto,heuredevoirs,done(ben,fairedevoirs,_),1,0,task)).


%%%%%%%%%%%%%%%%%%%
%% Acquaintances %%
%%%%%%%%%%%%%%%%%%%
% 7 MAGMA members
:- kassert(bel(naomjk,ami(naomjk,sylvie,1),1,0,conviction)).
:- kassert(bel(naomjk,ami(naomjk,kao,1),1,0,conviction)). 
:- kassert(bel(naomjk,ami(naomjk,wafa,1),1,0,conviction)). 
:- kassert(bel(naomjk,ami(naomjk,julie,1),1,0,conviction)). 
:- kassert(bel(naomjk,ami(naomjk,yves,1),1,0,conviction)). 
:- kassert(bel(naomjk,ami(naomjk,damien,1),1,0,conviction)). 
:- kassert(bel(naomjk,ami(naomjk,humbert,1),1,0,conviction)). 

:- kassert(trust(naomjk,_User)).

%%%%%%%%%%%%%%%%%%
%% Personality  %%
%%%%%%%%%%%%%%%%%%
:- kassert(seuil_expression(naomjk,0.4)).
:- kassert(seuil_ressenti(naomjk,0.2)).

