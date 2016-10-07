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

% November 2012
% actions with all their components: actiondef predicate
% predicates to check executability and feasability
% predicate to perform an action

% June 2013 : actually perform an action
% ie send the stimulus that it was performed, to all observers of this action

% TODO june2014 - declare an ontology of domain action using the actiondef predicate


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                              %%
%%    PERFORMANCE OF ACTIONS    %%
%%                              %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% perform an action in front of explicit additional watchers
perform(Agent,Action,Watchers) :-
    % time of performance
    instant(T),
    % print out action
    writeln('Agent '+Agent+' performs action '+Action+' in front of '+Watchers+' at time '+T),
    % observers: those agents that are necessarily aware of an action, eg hearer of a speech act
    observers(Action,Observers),
    % add the explicit watchers of the action
    merge(Watchers,Observers,ListAux),
    % add the performer of the action
    merge(ListAux,[Agent],List),
    % remove doubles so no agent receives the stimulus twice
    list_to_set(List,Set),
    % and send action stimulus to all concerned agents
    stimulus(Set,Action),
    % ground in this group that this action was performed by this agent at this time
    assert(grounded(Set,done(Agent,Action,T))).
           
% pB does not make sense for some agents to receive this stimulus (eg human agent...)
    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% OBSERVERS of an action %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% must not backtrack, so that perform does not backtrack

% observers of a loc act
observers(LocAct,[Hearer]) :-
    speech_act(IllocAct,LocAct),
    hearer(IllocAct,Hearer),!.

% the hearer is aware of a speech act he receives
observers(SpeechAct,[Hearer]) :-
    hearer(SpeechAct,Hearer), !.

% other actions have no observers, unless explicitly defined
% FIXME : where?
observers(_Action,[]) :- !.    



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                            %%
%%    SEMANTICS OF ACTIONS    %%
%%                            %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% SEMANTICS OF ACTIONS KNOWN BY ALL AGENTS %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% after an action was done, the agents know it was done
:- kassert(bel(_,after(I,Action,done(I,Action,_)),1,0,conviction)).


%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DEFINITION OF ACTIONS %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% an action should have
%%%   * a context of execution (physical conditions)
%%%   * preconditions (used in planning)
%%%   * positive effects (asserted in KB)
%%%   * negative effects (retracted from KB)

%% difference between epistemic and physical conditions and effects ?
%% ie are there effects asserted in the world and others in the KB ?
%% or does the agent observe the world after the action and gain knowledge about the physical effects?
%%% HOW to make a difference between epistemic conditions 
%         (the agent should know its position)
%%% and physical conditions 
% 	  (position 2 should be close enough to position 1)
%%% ???
%%% in particular in planning the agent cannot do anything to make 2 equal to 4
%%%                           but in can change its position to make it close enough


% actiondef(Agent,ActionName,Time,Context,Preconds,PosEffects,NegEffects)

:- dynamic(actiondef/7).

% abbreviations

context(ActionDef,Context) :- actiondef(_,ActionDef,_,Context,_,_,_),!.
preconds(ActionDef,Preconds) :- actiondef(_,ActionDef,_,_,Preconds,_,_),!.
poseffects(ActionDef,PosEffects) :- actiondef(_,ActionDef,_,_,_,PosEffects,_),!.
negeffects(ActionDef,NegEffects) :- actiondef(_,ActionDef,_,_,_,_,NegEffects),!.


%%%%%%%%%%%%%%%%%%%%
%% AGENT KNOW-HOW %%
%%%%%%%%%%%%%%%%%%%%

%% in each agent KB there will be can(agent,action)
%% listing all actions that the agent can do
%% ie knows how to do
%% only executability of these known actions will be checked during planning

:- dynamic(can/2).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% AGENT KNOWLEDGE OF ACTIONS %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% syntax: bel_action(A,P,D,T)

% if an agent can do a given action
% then find its actiondef to extract knowledge about preconditions and effects
% that will be used in planning

% preconditions
bel_action(_A,before(Agt,Action,Prop),1,T) :-
	% current instant
	instant(T),
	% this agent can do the action
	can(Agt,Action),
	% the action definition
	actiondef(Agt,Action,T,_Context,Preconds,_Poseffects,_Negeffects),
	% the proposition is in the preconditions of the action
	member(Prop,Preconds).

% positive effects
bel_action(_A,after(Agt,Action,Prop),1,T) :-
	% current instant
	instant(T),
	% this agent can do the action
	can(Agt,Action),
	% the action definition
	actiondef(Agt,Action,T,_Context,_Preconds,Poseffects,_Negeffects),
	% the proposition is in the positive effects of the action
	member(Prop,Poseffects).


% negative effects
bel_action(_A,after(Agt,Action,not(Prop)),1,T) :-
	% current instant
	instant(T),
	% this agent can do the action
	can(Agt,Action),
	% the action definition
	actiondef(Agt,Action,T,_Context,_Preconds,_Poseffects,Negeffects),
	% the proposition is in the positive effects of the action
	member(Prop,Negeffects).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  CONTEXT OF EXECUTION  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% generic predicate to check a list of conditions
%% FIXME difference with predicate valid ?
%% june2014 - replaced with valid
%%check_conds(ListConds) :-
%%	forall(member(OneCond,ListConds),OneCond),!.

check_conds(ListConds) :- valid(ListConds),!.

% context of execution are conditions that must be true for the action to succeed
% they are NOT preconditions that the agent must believe before wanting to do the action
% eg to move by one step from a place to another place, 
%         context imposes that the next place is indeed one step away
%         while preconditions impose that the agent believes it is at the start place and wants to go to the finish

executable(Agt,ActionName,_Time) :- 
	% find definition of action
	%actiondef(Agt,ActionName,Time,Context,_Preconds,_EffPos,_EffNeg),
	context(ActionName,Context),
	% agent can do the action
	can(Agt,ActionName),
	% context is ok
	check_conds(Context),!.

% if an action is executable but not feasible
% planning can build a plan to make it feasible	

% FIXME: feasible should depend on the agent...
feasible(Agt,ActionName,_Time) :-
	% find definition of action
	%actiondef(Agt,ActionName,Time,_Context,Preconds,_EffPos,_EffNeg),
	%preconds(ActionName,Preconds),
        precondition(_PlanningAgt,Agt,ActionName,Preconds,_Constraints),
	% preconditions are true
	check_conds(Preconds),!.



%%%%%%%%%%%%%%%%%%%%%%%%
%% EFFECTS of ACTIONS %%
%%%%%%%%%%%%%%%%%%%%%%%%

% check an action would have no NEW effect (already valid in any form, cf kbmgr)
has_no_new_effect(Agent,Action) :-
    %instant(T), - do NOT force current time or it does not work
    % empty list of effects that the agent does not already believe to be true - at ANY time
    findall(Effect,(establish(Agent,Action,Effect),not(valid_belief(Agent,Effect,_T))),[]).



%%%%%%%%%%%%%%%%%%%%
%% PERFORM METHOD %%
%%%%%%%%%%%%%%%%%%%%

%%%%%%%
%% generic assertion and retractation of a list of effects
%%%%%%%

assert_effects(ListPosEffects) :-
	forall(member(PosEffect,ListPosEffects),assert(PosEffect)),!.

retract_effects(ListNegEffects) :-
	forall(member(NegEffect,ListNegEffects),retract(NegEffect)),!.

% todo: use no fail predicate ?

% perform method checks the context, fails if conditions not verified
% simulate BDI reasoning in Prolog ?
% plans with context and perform method ?

% perform predicate has a list of witnesses as a parameter
% when the action succeeds, assert beliefs for all witnesses that the effects are true
% or have an inference rule that deduces effects from belief that action was done 
%                                                 and belief that action has this effect?

% perform(action) should also automatically send a stimulus with this action as content
% check pleiad_stimuli.pl

perform_old(Agt,ActionName,Time) :-
	% get definition
	actiondef(Agt,ActionName,Time,Context,Preconds,EffPos,EffNeg),
	% check context
	check_conds(Context),
	% check preconds
	check_conds(Preconds),
	% assert positive effects
	assert_effects(EffPos),
	% retract negative effects
	retract_effects(EffNeg).
	% assert action was done?
	
