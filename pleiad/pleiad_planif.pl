%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                                                       %%
%% 		PLEIAD Reasoning Engine			 %%
%%                                                       %%
%% Author : Jeremy Riviere, Carole Adam                  %%
%% Version : v1 - February 2012                          %%
%% Based on PLEIAD, developed by Carole Adam 2007        %%
%% Application to the CECIL project			 %%
%%							 %%
%%    ------- DO NOT DISTRIBUTE -------                  %%
%%                                                       %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% JUNE 2014
%% replaced old planif module with this one from Jeremy Inaffec
%%    - 1) New June 2013 : best goal from preferred source
%%    - 2) 2012 Jeremy planif module to replace old one

:- dynamic(my_plans/3).
:- dynamic(failed/2).


% goal_pref_src and main_intention moved to pleiad_delib - june2014


debug_planif :- add_debug_tag(plan),
                trace(findall_plans),
                trace(findall_plans_1g),
                trace(findall_plans_1a),
                trace(findall_estab_actions),
                trace(findall_false_preconds),
                trace(establishing_action),
                trace(establish),
                trace(precondition),
                trace(translate_precond),
                trace(false_preconditions),
                trace(remove_first_plan),
                trace(planning),
                trace(update_my_plans),
                trace(update_all_my_plans).

                
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 1) interface with reasoning        %%
%% several modes                      %%
%%   - plan until first feasible      %%
%%   - plan all intentions in agenda  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% plan until good
% in : agent, agenda : name of current agent and its agenda as computed by deliberation
% out : goodprop,newplans : the finally selected intention and the plans to achieve it (not empty)
plan_until_good(Agt,Agenda,GoodProp,NewPlans) :-
    debug_message(loop,['---Plan loop : reading main intention from agenda ',Agenda]),
    main_intention(Agt,Prop),
    debug_message(loop,['---Plan loop : main intention read in agenda of ',Agt,' is ',Prop,' - calling planner...']),
    
    %% FIXME
    % call external planner - asserts the plans... FIXME - should return them instead
    planning(Agt,Prop),
    % reads plans returned by planner
    my_plans(Agt,Prop,ListPlans),
    %% FIXME here
    
    debug_message(loop,['---Plan loop : planner computed list of plans ',ListPlans,' ']),
    % if empty, updates agenda, recursive call to replan
    ((ListPlans \= [],NewPlans=ListPlans,GoodProp=Prop) ;
        (update_agenda(Agt),
         agenda(Agt,NewAgenda),
         debug_message(loop,['---Plan loop : empty plans, updating agenda, recursive call...']),
         plan_until_good(Agt,NewAgenda,GoodProp,NewPlans))).
    % at the end of loop : came up with non-empty plans

% planning should NOT fail
plan_until_good(Agt,Agenda,_,[]) :-
    debug_message(loop,['---Plan loop : agent ',Agt,' exhausted agenda ',Agenda,' - no feasible intention, returns empty plan list']),!.

% plan all agenda : computes and asserts plans for all intentions in the agent's agenda
% should return the first non empty plans 
plan_all_agenda(Agt,Agenda,GoodProp,NewPlans) :-
            % for each intention in agenda
            forall(member([_Tag,intend(Agt,Prop,_Deg,_Time,_Src)],Agenda),
            % compute and assert plans to reach this intended proposition
            planning(Agt,Prop)),
            % then find first intention in agenda with non empty plans
            member([_AnyTag,intend(Agt,GoodProp,_AnyDeg,_AnyTime,_AnySrc)],Agenda),
            my_plans(Agt,GoodProp,NewPlans),NewPlans \=[],!.

% back up if plans were computed but no non-empty plan found
plan_all_agenda(Agt,[[main,intend(Agt,GoodProp,_,_,_)]|_Agenda],GoodProp,[]) :- !.
        


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2) JEREMY PLANIF               %%
%% from Inaffec/planning_rule.pl  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find all the plans to resolve the intention to have SubGoal %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% find one plan = find all plans, take first one - fails if no plan found
% PlanAgt = planning agent, Prop = proposition to establish, Action = plan of action to achieve Prop
% find_one_plan(PlanAgt,Prop,Action) :- findall_plans(PlanAgt,_,Prop,[Action|_ListPlans]).

% new version allowing backtrack
find_one_plan(PlanAgt,Prop,Action) :- findall_plans(PlanAgt,_,Prop,ListPlans),member(Action,ListPlans).

% PlanAgt = planning agent, ActAgt = acting agent
findall_plans(PlanAgt,ActAgt,SubGoal,ListPlans) :-
	findall_plans_1g(PlanAgt,ActAgt,SubGoal,ListPlans).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% auxiliary 1: find all plans that establish one subgoal: append (alternative) plans making each establishing action feasible


% recursive case: at least 1 action allows to establish the Subgoal
% recursive call of findall_plans_1g to make these actions feasible
findall_plans_1g(PlanAgt,ActAgt,Subgoal,ListPlans) :-
        debug_message(plan,['---Planning : planning agent ',PlanAgt,' is looking for plans for acting agent ',ActAgt,' to reach subgoal ',Subgoal,' ']),
	findall_estab_actions(PlanAgt,ActAgt,Subgoal,ListActions), ListActions \= [],
        debug_message(plan,['---Planning : list of establishing actions for ',Subgoal,' is ',ListActions,' ']),
	findall(UnPlan,
		(
			member(UneAction,ListActions),
			% recursively find plans that make each action feasible
			findall_plans_1a(PlanAgt,ActAgt,UneAction,DesPlansPourA),
                        debug_message(plan,['---Planning : Found the following plans ',DesPlansPourA,' to establish action ',UneAction,' ']),
			% append all establishing plans (OR) - each one is an alternative plan to make this action feasible
			member(UnPlanPourA,DesPlansPourA),
			% add the establishing action at the end of each
			append(UnPlanPourA,[UneAction],UnPlan)
		),
		% append all completed plans: each one is an alternative plan to the subgoal
		ListPlans).

% base case: no action allows to establish the SubGoal - return an empty list
findall_plans_1g(PlanAgt,ActAgt,Subgoal,[]) :- findall_estab_actions(PlanAgt,ActAgt,Subgoal,[]),
                                                debug_message(plan,['---Planning : planning agent ',PlanAgt,' knows NO action to establish subgoal ',Subgoal,' ']),
                                                !.	

                
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% auxiliary 2: find all plans that make one action feasible = plans to establish each precondition, appended with plans to establish the others

% first case: no false preconditions (all already true): there is an empty plan to establish this action
findall_plans_1a(PlanAgt,ActAgt,EstAct,[[]]) :-
                debug_message(plan,['---Planning : Building plan to make action ',EstAct,' feasible']),
                findall_false_preconds(PlanAgt,ActAgt,EstAct,[]),
                debug_message(plan,['---Planning : Action ',EstAct,' is already feasible']),
                !.
	
% recursive case: find all the false preconditions
% recursive call of findall_plans_1g to have the plan establishing the actions
findall_plans_1a(PlanAgt,ActAgt,EstAct,ListCombinedPlans) :-
	% trace,
	findall_false_preconds(PlanAgt,ActAgt,EstAct,ListPreconds),ListPreconds \= [],
        debug_message(plan,['---Planning : Action ',EstAct,' has the following false preconditions : ',ListPreconds,' ']),
	% recursively find all plans that make them true
	findall(UneListPlans,
		(
			member(UnePrecond,ListPreconds),
                        debug_message(plan,['---Planning : Trying to establish precondition ',UnePrecond,' ']),
			findall_plans_1g(PlanAgt,ActAgt,UnePrecond,UneListPlans)
		),
		ListListPlans),
	% and combine them to create combined plans
	combi_list_plans(ListListPlans,ListCombinedPlans),
        debug_message(plan,['---Planning : Combining ',ListListPlans,' into ',ListCombinedPlans,' ']).
	% notrace .

	
	
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Findall actions and preconditions given an action or a precondition %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% all actions (OR) that can establish a subgoal - returns a SET (no doubles)
findall_estab_actions(PlanAgt,ActAgt,SubGoal,ListActions) :-
    findall(Action,establishing_action(PlanAgt,ActAgt,Action,SubGoal),List),
    list_to_set(List,ListActions).

% all preconds (AND) that are needed to perform a subaction
% TODO: check that preconditions are false AND wouldn't be established by a previous action (performed before in the same plan) EX: go to ikea twice...
findall_false_preconds(PlanAgt,ActAgt,EstAct,ListPreconds) :-
    findall(Precond,false_preconditions(PlanAgt,ActAgt,EstAct,Precond,_Constraints),List),
    list_to_set(List,ListPreconds).

% special case: any action establishes that this action is done
establishing_action(_,ActingAgt,Action,done(ActingAgt,Action,_T)).

% any action establishes that the planning agent believes this action to have been done
% FIXME: should the planning agent really be observer of ALL actions performed? should be the hearer of the speech act?
establishing_action(PlanningAgt,Speaker,Action,bel(PlanningAgt,done(Speaker,Action,_T),_D1,_T1,_Src)).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Establishing speech act %%%

% new generic establishing speech act - cf language_jeremy.pl
% FIXME raisonner plutot sur les after - generes par assert_language au debut a partir des send_effects et receive_effects

establishing_action(PlanningAgt,ActingAgt,SpeechAct,bel(PlanningAgt,Prop,1,T,communication)) :-
    debug_message(demoz,['establishing action case 1 - before']),
    establish(PlanningAgt,SpeechAct,Prop),instant(T),
    speaker(SpeechAct,ActingAgt),
    debug_message(demoz,['establishing action case 1 - after']).

% ou l'inverse ? FIXME - un speech act etablit une prop si son effet est que PlanAgt croit cette prop
establishing_action(PlanningAgt,ActingAgt,SpeechAct,Prop) :-
    debug_message(demoz,['establishing action case 2 - before']),
    establish(PlanningAgt,SpeechAct,bel(PlanningAgt,Prop,_Deg,_Time,_Src)),
    speaker(SpeechAct,ActingAgt),
    debug_message(demoz,['establishing action case 2 - after']).

% ou sans ajouter de bel - FIXME
establishing_action(PlanningAgt,ActingAgt,SpeechAct,Prop) :-
    debug_message(demoz,['establishing action case 3 - before']),
    establish(PlanningAgt,SpeechAct,Prop),
    speaker(SpeechAct,ActingAgt),
    debug_message(demoz,['establishing action case 3 - after']).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Establishing "physical" action %%%		
% general case
establishing_action(PlanningAgt,ActingAgt,action(ActingAgent,Action),SubGoal) :-
                        debug_message(demoz,['establishing action case 4 - before']),
			believe(PlanningAgt, Cond -> after(ActingAgt,action(ActingAgent,Action),SubGoal),_,0),valid(Cond),
                        debug_message(demoz,['establishing action case 4 - after']).

% without conditions
establishing_action(PlanningAgt,ActingAgt,action(ActingAgent,Action),SubGoal) :-
                        debug_message(demoz,['establishing action case 5 - before']),
			believe(PlanningAgt, after(ActingAgt,action(ActingAgent,Action),SubGoal),_,0),
                        debug_message(demoz,['establishing action case 5 - after']).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% FIND and TRANSLATE PRECONDITIONS OF ACTIONS  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% find all preconditions of an action
% warning: the planning agent is not necessarily the one performing the action
precondition(PlanningAgt,ActingAgt,Action,Precond,[]) :- 
			believe(PlanningAgt, before(ActingAgt,Action,Precond),_,_T),
			not(is_list(Precond)).
			
% conditional preconditions
precondition(PlanningAgt,ActingAgt,Action,Precond,[]) :-
                        believe(PlanningAgt,Prem->before(ActingAgt,Action,Precond),_,_T),
                        valid(Prem),
                        not(is_list(Precond)).
			
precondition(PlanningAgt,ActingAgt,Action,Precond,Constraints) :-
			instant(T),
			believe(PlanningAgt,Prem -> before(ActingAgt,Action,[Precond,Constraints]),_,_T1),
			believe(PlanningAgt,Prem,_,T).
						
			
precondition(PlanningAgt,ActingAgt,Action,Precond,Constraints) :-
			believe(PlanningAgt,before(ActingAgt,Action,[Precond,Constraints]),_,_T).
			
			
% TODO july2014 - supprimer cette translation (bel_prob remplace par bel deja)
% add an intermediary level where a bel_prob, believe or prob precondition is translated into a bel
% so that it matches the postconditions of establishing actions (asserting bel(...) )
% translate_precond2(PlanAgt,ActAgt,Action,bel(PlanAgt,P,D,T,_Src),Constraints,bel_prob(PlanAgt,P,D,T)) :-
%			precondition(PlanAgt,ActAgt,Action,bel_prob(PlanAgt,P,D,T),Constraints).
%
%translate_precond2(PlanAgt,ActAgt,Action,bel(PlanAgt,P,D,T,_Src),Constraints,prob(PlanAgt,P,D,T)) :-
%			precondition(PlanAgt,ActAgt,Action,prob(PlanAgt,P,D,T),Constraints).
%
%translate_precond2(PlanAgt,ActAgt,Action,bel(PlanAgt,P,D,T,_Src),Constraints,believe(PlanAgt,P,D,T)) :-
%			precondition(PlanAgt,ActAgt,Action,believe(PlanAgt,P,D,T),Constraints).
%									
%translate_precond2(PlanAgt,ActAgt,Action,Precond,Constraints,Precond) :-	
%			precondition(PlanAgt,ActAgt,Action,Precond,Constraints),
%			%Precond \= bel_prob(_,_,_,_),
%			Precond \= prob(_,_,_,_),
%			Precond \= believe(_,_,_,_).
%		
% for debug
%translate_precond(PlanAgt,ActAgt,Action,Precond,Constraint) :-
%                    translate_precond2(PlanAgt,ActAgt,Action,Precond,Constraint,Translated),
%                    display_message(['Planning agent =',PlanAgt,'; Acting agent =',ActAgt,'; Action =',Action,'; precond after translation = ',Precond,'; before translation = ',Translated,'; constraint = ',Constraint,' ']).
                
% find all preconditions yet to be established
false_preconditions(PlanningAgent,ActingAgent,Action,Precond,Constraints) :-
                        % new JULY2014 - not necessary to translate preconditions anymore thanks to new validity predicate (cf kbmgr/valid_belief)
                        % -> therefore using precondition instead of translate_precond below (cf draft july2014 for old version)
			findall([Prec,Cons], precondition(PlanningAgent,ActingAgent,Action,Prec,Cons), ListPrecCons),
			% translate_precond(PlanningAgent,ActingAgent,Action,Precond,Constraints),
			member([Precond,Constraints],ListPrecCons),
			inverse_plan(Constraints,ConstraintsInverse),
			% inverse the list of preconditions because they are checked in reverse order
			% while needs to check them in order in case the first one helps instantiating the next ones
			false_constraint_precond(Precond,ConstraintsInverse).
			
			
check_constraint_precond(Cond,[]) :- valid(Cond).

check_constraint_precond(Cond,[A|Constraints]) :- check_constraint_precond(Cond,Constraints),A .

false_constraint_precond(Cond,Constraints) :- 
	not(check_constraint_precond(Cond,Constraints)).
	
        
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MY PLANS UPDATE PREDICATES %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% remove first plan (called if unfeasible / first action fails)
remove_first_plan(Agt,Prop,FirstPlan) :- retract(my_plans(Agt,Prop,[FirstPlan|ListPlans])),
                                assert(my_plans(Agt,Prop,ListPlans)).

	
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% How to execute the main intention? Planning %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FIXME warning this update is never called !

update_all_my_plans(Agt) :- instant(T),
			    %no need to sort intentions here
                            %vecteur_intend(Agt,T,Intend_List),
                            % for each intention of the agent that has an existing (asserted) plan library
                            forall((intend(Agt,P,_D,T,_Src),my_plans(Agt,P,ListPlan)),
                            % update this plan library and re-assert it
                                (update_my_plans(ListPlan,NewPlans),
                                 assert_my_plans(Agt,P,NewPlans),
                                 debug_message(plan,['---Update all plans: agent ',Agt,' updates existing plans ',ListPlan,' for prop ',P,' to new plans ',NewPlans,' '])
                                )),!.

                                
update_all_my_plans(_Agt) :- !.							





%%%%%%%%%%%%%%%%%%%%%%%%
%% PLANNING PREDICATE %%
%%%%%%%%%%%%%%%%%%%%%%%%

%%%%% Get plans already computed for this intention - do not replan if plans are already available

%% this case should not happen... FIXME july2014
%% I have done the last action of my plan
%% planning(Agt,Prop) :- my_plans(Agt,Prop,[[]|_]),
%%                                % first plan is empty
%%				debug_message(plan,['---Planning : intention ', Prop , ' first available plan is empty ']),!.   % -- No more plans after update
%%				%retract_intention(Agt,Prop), planning(Agt).
%%                                % FIXME : only first plan is empty, should switch to next plan

% There is already a list of plans computed for this intention
% There are still actions to do in current plan for this intention
planning(Agt,Prop) :- my_plans(Agt,Prop,ListPlan),
                                % first plan of the list is not empty
				ListPlan \= [[]|_],
				debug_message(plan,['---Planning : intention ', Prop , ' updated existing plans ', ListPlan,' ']),!.
					 
%%%%% compute new plans to achieve this intention, and assert them
planning(Agt,Prop) :-
                            debug_message(plan,['---Planning : intention ',Prop,' is new, no plans available yet, starting planning...']),
                            % findall plans - NOT with this planning agent as the acting agent ! do not instantiate ActAgt or it forces only actions of this agent WARNING
			    findall_plans(Agt,_AnyActingAgt,Prop,ListPlans),
                            assert_my_plans(Agt,Prop,ListPlans),  % [Plan|ListPlans]
			    %assert_goals_related_to_my_plans(Agt,Prop,Plan), % Plan is first plan of list
			    debug_message(plan,['---Planning : for new intention ', Prop , ' computed plans:', ListPlans, ' ']),!.
		
%%%%%% new june2014 - last default case : assert that there are no plans for this intention
%%%%%% it is not the job of planning to switch intention or anything else
% should be a subcase of previous one now
planning(Agt,Prop) :- assert_my_plans(Agt,Prop,[]),
                        debug_message(plan,['---Planning (bug) : new intention ', Prop , ' but no available plans... ']),!.

		

%%%%
%% for debug purposes
%%%%
debug_planning(Agt,Prop,Plans) :-
            add_debug_tag(plan),
            retractall(my_plans(Agt,Prop,_AnyPlans)),
            planning(Agt,Prop),
            my_plans(Agt,Prop,Plans).


							  	
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% My plans manipulation %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

update_my_plans(OldListPlan,ListPlansUpdated) :- 
					removed_last_achieved_action(OldListPlan,FirstUpdate),
					drop_unfeasible_plans(FirstUpdate,ListPlansUpdated).
					
					
drop_unfeasible_plans(OldListPlan,ListUpdated) :- 
				% find unfeasible plans (those whose first action has false preconditions)
				plans_with_unfeasible_first_action(OldListPlan,UnfeasiblePlans),
				% drop them (remove them from my_plans)
				remove_some_plans(UnfeasiblePlans,OldListPlan,ListUpdated),!.
				
drop_unfeasible_plans(ListUpdated,ListUpdated) :- !.		

% remove this plan because the first action has some false preconditions, and the previous action is successful
plans_with_unfeasible_first_action(ListAllPlans,ListUnfeasiblePlans) :-  
									previous_action_successful,
									findall([Tag,[Action|Plan]],(
									member([Tag,[Action|Plan]],ListAllPlans),
									false_preconditions(_PlanningAgent,_ActingAgent,Action,Precond,_Constraints),
									print('\n'+ Action + 'has false precond :'+ Precond)
									),
									ListUnfeasiblePlans),!.		
									
% do not remove the plan because the previous speech act is not yet successful
% previous_action_successful defined in pleiad_language_jeremy.pl
plans_with_unfeasible_first_action(_ListAllPlans,[]) :- not(previous_action_successful).			

% special case: 
remove_some_plans(PlansToRemove,PlansToRemove,[[]]) :- !.

% tool : remove list1 from list2
remove_some_plans(PlansToRemove,AllPlans,NewList) :-
				findall(Plan,(member(Plan,AllPlans),not(member(Plan,PlansToRemove))),NewList),!.

removed_last_achieved_action(OldListPlans,ListPlansUpdated) :- 
				% get the last action performed
				laststimulus(_Agt,Action),
				% remove this action from the head of all the plans
				remove_first_action(Action,OldListPlans,ListPlansUpdated),!.

removed_last_achieved_action(ListPlansUpdated,ListPlansUpdated) :- !.
					
%% The last action succeeded: have to remove this action from all the plan's heads
remove_first_action(HeadAction,ListPlans,NewListPlans) :- 
			findall([Tag,NewPlan],(member([Tag,Plan],ListPlans),remove_head_action(Plan,HeadAction,NewPlan)),NewListPlans).
			
% auxiliary that removes the head action of a plan if it is the wanted action
remove_head_action([Action|Plan],Action,Plan) :- !.
% if the plan does not start with the requested action it stays unchanged
remove_head_action(Plan,_Action,Plan).


%% perform_next_action, what_expected, etc : moved to pleiad_scheduling - june2014



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 0) Initialise list of PLANS  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% my_plans can contain either not initialed plans (asserted by planning)
%% or already initialised plans (asserted at end of planning loop by reasoning)
%% need to check that plans are not initialised yet before initialising them

already_initialised([[ongoing,_Plan]|_]) :- !.
already_initialised([[new,_Plan]|_]) :- !.
already_initialised([[failed,_Plan]|_]) :- !.

% initialise all plans tags (1st element of the list) to 'new'
% special case so that planning does not fail if no plans 
initialise_plans([],[]) :- !.

% special case if plans already initialised: do not do it again
initialise_plans(Plans,Plans) :- already_initialised(Plans),display_message(['---Init plans : plans already initialised']),!.

% normal case
initialise_plans(MyPlansNI,[[ongoing,FirstPlan]|ListInitPlans]) :-
        % remove duplicate plans
        list_to_set(MyPlansNI,[FirstPlan|ListPlans]),
        % append tag ongoing at the start of first plan
        % nth0(0,InitFirstPlan,ongoing,FirstPlan),
        % append tag new at the start of all plans except first one
        findall([new,Plan],member(Plan,ListPlans),ListInitPlans).


%%%%
%% PLANS
%%%%

%% assert my plans to do the main intention, and assert my goals
assert_my_plans(Agt,Prop,ListPlan) :- 
	ifpossible(retract(my_plans(Agt,Prop,_)),
	assert(my_plans(Agt,Prop,ListPlan))),!.	 
													

initialise_asserted_plans(Agt) :-
    forall(retract(my_plans(Agt,Prop,PlansNI)),
           (initialise_plans(PlansNI,PlansI),
            assert(my_plans(Agt,Prop,PlansI)))
           ).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3) planning tools      %%
%% from Inaffec/tools.pl  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plans Manipulation: combine plans %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% combinaison de tous les plans pour atteindre une precondition 
%% avec tous les plans pour en atteindre une autre
%% production de tous les plans sequences combines

combi_2listsof_plans([],_L2,[]) :- !.
combi_2listsof_plans([E1|L1],L2,L) :-
	findall(LE,(member(L,L2),append(E1,L,LE)),L3),
	combi_2listsof_plans(L1,L2,L4),
	append(L3,L4,L).

% combinaison d'un plan dans chaque liste de plans (element d'une liste de listes de plans)
% chaque liste de plans contient des plans alternatifs
% la liste de listes de plans represente une sequence (il faut faire un plan dans chaque liste)
% ce predicat construit une liste de plans obtenus en combinant un plan de chaque liste de plans

% cas de base: 1 seule liste de plans: c'est le resultat final
combi_list_plans([ListPlans],ListPlans) :- !.

% cas general: combiner le 1e avec le 2e, puis recursivement le resultat avec le reste
combi_list_plans([ListPlans1,ListPlans2|ListListPlans],ListCombinedPlans) :-
	% combine 2 first plans
	combi_2listsof_plans(ListPlans1,ListPlans2,ListCombined1),
	% recursively combine the result with the rest
	combi_list_plans([ListCombined1|ListListPlans],ListCombinedPlans).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MY PLANS MANAGEMENT - from Jeremy  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% from belief_manager.pl moved to pleiad_kbmgr.pl

        						
%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Goals degree %%%%%%%%
%% from Jeremy tools.pl %%
%%%%%%%%%%%%%%%%%%%%%%%%%%
% compute_goal_degree from Jeremy removed July2014 (cf drafts_july2014.txt)
% FIXME june2014 a quoi servent ces degres et d'ou ils sortent ???
% pourquoi le param SpeechAct ne sert a rien dans le calcul / l'intention consideree est independante de l'action


%%%%
%% GOALS LINKED TO MAIN INTEND
%%%%   

%% assert the goals linked to my main intention (from Jeremy)
%% ie assert goals to establish false preconditions of first action

% warning Nov 2012 : no base case for empty list
% warning MAY 2013 : goals now have an instant as last parameter
% warning june 2014 : goals now have a source as last 5th parameter

% List of one action
%assert_goals_related_to_my_plans(Agt,Prop,[FirstAction]) :-  
%	findall_false_preconds(Agt,_ActAgt,Prop,ListPreconds),
%	% compute goal degree in Jeremy's tools.pl
%	compute_goal_degree(Agt,FirstAction,Deg),
%	% assert goal with this degree
%	forall(member(Precond,ListPreconds),assert_goal(Agt,Precond,Deg,T,Src)),!.									   
% list of at least 2 actions
%assert_goals_related_to_my_plans(Agt,Prop,[FirstAction,SecondAction|Plan]) :-
%	findall_false_preconds(Agt,_ActAgt,SecondAction,ListPreconds),
%	compute_goal_degree(Agt,FirstAction,Deg),
%	forall(member(Precond,ListPreconds), assert_goal(Agt,Precond,Deg,T,Src)),
%	assert_goals_related_to_my_plans(Agt,Prop,[SecondAction|Plan]).														

% TODO june2014 - added T and SRC to goals but not instantiated. Should get them from the intend(Agt,Prop,Deg,T,SRC)
% so that it depends on the intention that Prop, rather than on the main intention as was the case previously
% also change compute_goal_degree similarly

