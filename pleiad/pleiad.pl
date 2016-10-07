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




%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% OPERATEURS DYNAMIQUES %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%:- abolish(bel/5).

kdynamic :-
  dynamic(rule/5),   		% new October 2012
  dynamic(believe/4),
  dynamic(intend/5),
  dynamic(goal/5), % new June 2013
  dynamic(condgoal/6),  % new July 2014
  dynamic(focus/4), 
  dynamic(bel/5),
  dynamic(grounded/2),
  dynamic(desire/3),
  dynamic(undesire/3),
  dynamic(emotion/5),
  dynamic(ami/3),
  dynamic(ennemi/3),
  dynamic(ideal/3),
  dynamic(unideal/3),
  dynamic(personality/3), 
  dynamic(tag/3),
  dynamic(laststimulus/2),
  multifile(bel/5),
  kdyn_language.


kdyn_language :-
    dynamic(speaker/2),
    dynamic(hearer/2),
    dynamic(illoc_force/2),
    dynamic(prop_content/2),
    dynamic(speech_act/2), % new July 2013
    multifile(speaker/2),
    multifile(hearer/2),
    multifile(illoc_force/2),
    multifile(prop_content/2).


:- assert(committed(noone,nothing,0,0)).
:- assert(abandoned(noone,nothing,0)).

% abolish dynamic predicates to reset
kabolish :-
    abolish(bel/5),
    abolish(intend/5),
    abolish(goal/5),
    abolish(condgoal/6),
    abolish(grounded/2),
    abolish(instant/1),
    abolish(laststimulus/2),
    abolish(list_loaded_agents/1).
    % to be completed if necessary
    
% retract dynamic predicates to reset
kretractall :-
    retractall(bel),
    retractall(intend),
    retractall(goal),
    retractall(condgoal),
    retractall(grounded),
    retractall(instant),
    retractall(laststimulus),
    retractall(list_loaded_agents/1).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% TEST OF DEMO RULES SYNTAX  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% premises should now be a list!
test_demo_rules :-
    forall(
           (rule(I,_A,_T,Prem,_Cci),not(is_list(Prem))),
           print('rule '+I+' has bad syntax')
        ).


% initially: empty list of tags
:- assert(kdebugtags([])).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% INCLUSION DES FICHIERS DU PROJET %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% for each version, should also define the active rules (cf pleiad_demo and pleiad_reasoning)
% to activate only the rules that are needed
% for example rule 25 for dialogue only in version benoit
% warning: the active_rules predicate is bypassed in pleiad_reasoning at the moment

% version standard
% 20 pleiad files + 1 KB

% reset
reset :-
    % abolish and retract all predicates (including instant and loaded agents)
    kabolish, kretractall, kdynamic,
    % remove all loaded agents (needs predicates defined in pleiad_write so load file first)
    % [pleiad_write],remove_all_loaded_agents,
    % reassert initial instant (retracted above but needed)
    assert(instant(0)),
    % reassert empty list of loaded agents
    assert(list_loaded_agents([])).

pleiad :- reset,
  [pleiad_tools],
  [pleiad_debug],               % new June 2014
  [pleiad_relations],		% new October 2012
  [pleiad_kbmgr],  		% new November 2012
  [pleiad_grounding],           % new July 2014
  [pleiad_tendencies],       % reloaded with each agent
  [pleiad_focus],
  [pleiad_decay],
  [pleiad_perso],
  [pleiad_time],
  [pleiad_stimulig],
  % [pleiad_perception],		% new October 2012
  [pleiad_write],
  [pleiad_reasoning],		% also compiles pleiad_demo
  [pleiad_delib],
  [pleiad_scheduling],
  [pleiad_planif],
  [pleiad_intentions],
  [pleiad_actions],          % reloaded with each agent
  [kbs/commonsense],         % reloaded with each agent
  [kbs/laws],             % reloaded with each agent
  test_demo_rules.           % tests 

% rk: pleiad_write/load_kb reloads the KB of an agent but also
% its commonsense beliefs, laws, known actions, action tendencies

kaolla :- pleiad,
        [pleiad_emotions].   % also compiles pleiad_expression

% emiliano for coping
emiliano :-
        kaolla,
        [pleiad_coping_emiliano],  % new version for paams work with emiliano
        load_kb(robot), load_kb(killua), test_demo_rules.


% jeremy for multimodal conversational language
% assert a null threshold for feeling and expressing emotions for all agents (overwritten by agent personality)
jeremy :- pleiad,debug_message(compil,['basic pleiad compiled successfully']),
    %% [pleiad_speechacts_jeremy], %% components speaker/hearer/cprop/force
  [pleiad_speechrules_jeremy], %% protocol
  [pleiad_speechlib_jeremy], %% library of speechacts
  assert(seuil_ressenti(Agent,0)),
  assert(seuil_expression(Agent,0)),
  [pleiad_emotions_jeremy],    %% cecil emotions, feeling and expressing threshold, scherer's checks
  debug_message(compil,['jeremy pleiad compiled successfully']),
  %load_kb(holiday),assert_language,!.
  load_kb(toto),assert_language,!.

% debug planif
nao :- pleiad,debug_message(compil,['basic pleiad compiled successfully']),
	% debug tags to trace demo, planning, etc
        add_debug_tags([plan,loop,schedule,delib,demo,lang]),
        [pleiad_speechrules_jeremy], %% protocol
        [pleiad_speechlib_jeremy], %% library of speechacts
        [pleiad_emotions_jeremy], %% cecil emotions, feeling and expressing threshold, scherer's checks
        debug_message(compil,['jeremy (nao) pleiad compiled successfully']),
        load_kb(naomjk),
	% position (physique) initiale
	assert(position(naomjk,mjk)),
	% multimodal language : translate send_condition to before, send_effect to after
	assert_language,!.


% domi for shame and pride
domi :- kaolla,
        [pleiad_shame_domi],  % new version for RIA work with Domi
        load_kb(tom),load_kb(mamtom),test_demo_rules.


% benoit for new semantics of speech acts with locutionary and illocutrionay acts
% and insincere agents
benoit :-  kaolla,
        [pleiad_speechacts_benoit],  % new version for JAAMAS work with Benoit
        load_kb(coco),test_demo_rules.


