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


% IDEAS October 2012
% 1) new aspect of personality = positivity
% affects the tendency to express rather the (most intense) 
% positive or the negative emotion
% cf pleiad_expression.pl

% 2) other new aspect = rancorousness / (un)forgivingness
% affects the tendency to feel (and express) reproach-like emotions 
% towards responsible agent
% cf pleiad_expression.pl

% 3) tendency to panick more or less easily
% ie threshold of emotion intensity that gets the agent into panick mode 
% (where only the most activated mental attitudes are considered in reasoning)

%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%
%%% from decay.pl %%%
%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%

%% 26 July 2007

:- dynamic(personality/2).

% each agent will have its own vector given in its personality(V)
% here for debug purposes

personality(test,[joy=1,sadness=1,hope=1,fear=1,satisfaction=1,disappointment=1,
                  relief=1,fearconfirmed=1,pride=1,shame=1,admiration=1,reproach=1,
                  gratitude=1,anger=1,gratification=1,remorse=1,happyfor=1,resentment=1,
                  sorryfor=1,gloating=1]).

% this is a default predicate (for debug purposes) to give a default personality to an agent
set_perso(Agt,hypersensible) :- nofail(retract(personality(Agt,_))),
						assert(personality(Agt,[])).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% perso.pl %%% perso.pl %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% personality module

%% the agent's personality comprises
% focus decay speed (later: depending on emotion valence)
% sensibility to each emotion for its expression
% order of preference on coping strategies -> ok
%    should depend on emotion (ie the agent prefers a different strategy for each different emotion)
% computation of activation threshold needed to use a mental attitude, depending on max emotion 
%  intensity at that time (ie degree of panic)
% action tendencies induced by each emotion (eg fight or flight)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%                                                          %%%%
%%%% COMPUTATION OF ACTIVATION THRESHOLD                      %%%%
%%%%    AS A FUNCTION OF EMOTION INTENSITY                    %%%%
%%%%                                                          %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% TODO: move it to pleiad_reasoning.pl and actually use it

% FIXME : predicate emo_intens not defined
% use emo_maxintens(Agt,T,E) defined in pleiad_coping instead

% returns minimal threshold of activation of mental attitudes to be taken into account in reasoning
% by agent Agt, depending on his personality and intensity of his emotion
% seuil(Agt,Seuil_rr) :- 
%		%% get intensity degree of most intense emotion
%		emo_intens(Agt,emotion(Agt,_,_,Deg_intens,_),_),
%		%% compare it with the threshold stored in agent personality
%		perso_seuil_raiso(Agt,Deg_intens,Seuil_rr).

                
% 1) normal reasoning
% uses all mental attitudes (focus >=0) if intensity of emotion <=0.5
perso_seuil_raiso(_,Intens,0) :- Intens =< 0.5.
% 2) accelerated reasoning
perso_seuil_raiso(_,Intens,0.5) :- Intens >0.5,Intens<0.8.
% 3) panicked reasoning
% if intensity of emotion is over 0.8 the focus must be over 0.8 to use the mental attitude
perso_seuil_raiso(_,Intens,0.8) :- Intens >=0.8.

% TODO add a bias based on personality (tendency to panick more or less easily)
% biais_raiso(agt,degbiais)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                                        %%%
%%% PREFERENCE ORDER ON COPING STRATEGIES  %%%
%%%                                        %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Assert default order for all agents
% because in pleiad_coping.pl the agent must have a preference order 
% otherwise no strategy is applied at all
:- kassert(prefcop(_,[active_coping(_),seek_material_support(_),denial(_),positive_reinterpretation(_),mental_disengagement(_),shift_responsability(_)])).

% in each agent's KB, predicate prefcop(agent,vector)
% returns the ordered vector of preferred coping strategies of the agent



%%%
% personal prefcop: to be used for coping
% checks if there exists a personal list and returns it
% otherwise returns the default list
%%% 

% returns the list of personal coping preferences of an agent (false if none)
% ie a list that is not shared with a (not existing) default agent zzzz
% ie it is not the default coping preferences asserted just above
personal_prefcop(Agt,List) :- prefcop(Agt,List),not(prefcop(zzzz,List)),!.

% default case: returns the default list rather than failing
personal_prefcop(Agt,List) :- prefcop(Agt,List).






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                                      %%%
%%% SENSIBILITY for EMOTION EXPRESSION   %%%
%%%                                      %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% TODO October 2012
% cf each agent's KB + expression.pl + emotion.pl


% each agent only feels an emotion if its intensity > a threshold
% threshold is specific to this emotion and this agent


% 2nd threshold for expressing that particular emotion by this particular agent


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                          %%%
%%%    EMOTION DECAY SPEED   %%%
%%%                          %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% each agent feels an emotion for a more or less long time before it disappears
% depending on its personality
% for ex: optimism makes positive emotions last longer and negative emotions shorter





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                                      %%
%% RELATIVE IMPORTANCE OF GOAL SOURCES  %%
%%                                      %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% sources are : task (reach personal objective), social (respect social norms), emotion (express emotions)

% default order: emotion, then social, then task
:- kassert(prefgoal(_,[emotion,social,task])).

