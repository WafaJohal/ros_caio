%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                                                       %%
%% PLEIAD : ProLog Emotional Intelligent Agents Designer %%
%%                                                       %%
%% Author : Carole Adam                                  %%
%% Version : v7 - June 2014                              %%
%%                                                       %%
%%    ------- DO NOT DISTRIBUTE -------                  %%
%%                                                       %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% TODO june2014 - rename perform_next_action en get_next_action, et enlever le stimulus a la fin
%% faire un predicat perform_action a la place, qui recoit l'action a performer, et envoie le stimulus
%% TODO 2 get_next_action devrait recevoir la liste des plans directement, pour eviter de recalculer l'intention et les plans a chaque etape !!

%% TODO should isolate get_next_action (scheduling)
%%   -- that backtracks when plan fails, tries next plan instead of failing, until no more plans available
%% from perform_action(Action) that receives the action to perform (decided by scheduling)
%%   -- and actually performs it, asserts consequences, sends stimulus, informs observers, etc


%%%%%%%%%%%%%%%%%%%%%%%%
%% DYNAMIC PREDICATES %%
%%%%%%%%%%%%%%%%%%%%%%%%

:- dynamic(expected/3).


%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MAIN ACTION PROGRAM  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%

% debug only - see schedule phase in pleiad_reasoning
%mainprog_schedule(Agt,Action) :-
%        get_plans(Agt,ListPlans),
%        initialise_plans(ListPlans,ListInitPlans),
%        schedule_next_action(ListInitPlans,Action),
%        perform_action(Action).


%%%%%%%%%%%%%%%%%%
%% SWITCH PLAN  %%
%%%%%%%%%%%%%%%%%%
       
% mark ongoing plan as failed, then find next plan still marked as new and make it ongoing
% prints failed plan (for debug and self-explaining purposes)
switch_plan(ListPlans,NewListPlans) :-
        % remove ongoing plan from the list of plans
        select([ongoing|OngoingPlan],ListPlans,TmpList1),
        % replace ongoing tag with failed tag and insert failed plan in the tmp list
        nth0(0,TmpList2,[failed|OngoingPlan],TmpList1),
        debug_message(schedule,['---Scheduling : dropped failed plan ',OngoingPlan]),
        % find first new plan and remove it from tmp list
        select([new|NewPlan],TmpList2,TmpList3),
        % make it ongoing (fails if no more new plan), insert it back into tmp list
        nth0(0,NewListPlans,[ongoing|NewPlan],TmpList3),
        debug_message(schedule,['---Scheduling : switched to next new plan ',NewPlan]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialise ONE plan for performing it  %%
%% and monitoring performance             %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% predicate my_schedule for agent's current plan
% action tags : todo, next (all other), failed, skipped, performed
assert_current_plan(Agt,NewPlan) :-
        % retract previous current plan - does not fail if nothing to retract
        retractall(current_plan(Agt,_OldPlan)),
        % assert new one
        assert(current_plan(Agt,NewPlan)).
        
% initialise plan as found in list of plans - initialised plan will be asserted as current
initialise_one_plan([FirstAction|PlanFromAgd],[[todo,FirstAction]|InitPlan]) :-
        findall([next,Action],member(Action,PlanFromAgd),InitPlan).
        
        
        
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% SKIP TODO ACTION         %%
%% if no need to perform it %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% auxiliary: change tag of todo action, insert it with its new tag at end of plan
% WARNING only works if todo action is head of plan
chgtag_todo_action([[todo,TodoAction]|Plan],NewTag,NewPlan) :-
        % insert todo action changed to NewTag at end of plan
        insert_last_position([NewTag,TodoAction],Plan,NewPlan).

% promote first next action to todo
% mark next action as todo, if there is one, FIXME fail if exhausted? but means plan success
promote_next_action_todo([[next,Action]|Plan],[[todo,Action]|Plan]) :- !.
        %%old version
        % extract next action from Plan  - cut to select only first one (do actions in order)
        % select([next,NextAction],Plan,RestOfPlan),!,
        % insert next action with tag toto at start of plan
        % insert_first_position([todo,NextAction],TmpPlan,NewPlan),!.



% skip an action (the todo action)
% because it is an action from another agent
% or because it will have no new effect (all effects already hold, action useless)
skip_todo_action(OldPlan,NewPlan) :-
        chgtag_todo_action(OldPlan,skip,TmpPlan),
        % promote next action if available
        (promote_next_action_todo(TmpPlan,NewPlan);
         % or if skipped the last todo action - do nothing, return exhausted plan
         NewPlan=TmpPlan).

% to prevent failure - if no todo action to skip, do nothing but succeed
skip_todo_action(Plan,Plan) :- !.
        
     
%% check if need to perform or skip action
%% skipORperform(Action)

skippable_todo_action(Agent,[[todo,Action]|_Plan]) :-
    skippable_action(Agent,Action),!.
    

% case 1a : the author of the (physical) action is another agent
skippable_action(TheAgent,action(ActionAgent,ActionName)) :-
                    ground(ActionAgent),ActionAgent\=TheAgent,
                    debug_message(schedule,['---Schedule : agent ',TheAgent,' skips physical action ',ActionName,' performed by other agent ',ActionAgent,' ']),!.

% case 2b : the speaker of the speech act is another agent
skippable_action(TheAgent,speechact(Force,Speaker,_Hearer,Cprop)) :-
                    ground(Speaker),Speaker\=TheAgent,
                    debug_message(schedule,['---Schedule : agent ',TheAgent,' skips language action ',Force,'(',Cprop,') performed by other speaker ',Speaker,' ']),!.

% case 2 : the action has no new effect (cf pleiad_actions)
skippable_action(Agent,Action) :-
                has_no_new_effect(Agent,Action),
                debug_message(schedule,['---Schedule : agent ',Agent,' skips action ',Action,' with no new effects ']),!.
        
%%%%%%%%%%%%%%%%%%%%%%%%
%% update my schedule %%
%%%%%%%%%%%%%%%%%%%%%%%%

update_my_schedule(Agt) :-
    % retractall does not fail if nothing to retract, but does not instantiate schedule in any case
    % retract instantiates Schedule but fails if nothing to retract
    retract(my_schedule(Agt,Schedule)),
    ((skippable_todo_action(Agt,Schedule),skip_todo_action(Schedule,NewSchedule)),NewSchedule=Schedule).
    % TODO ongoing july2014
    % get result of last action from performance
    
update_my_schedule(Agt) :-
    debug_message(schedule,['---Scheduling: agent ',Agt,' has no schedule to update']),!.
        
%%%%%%%%%%%%%%%%%%%%
%% 1) scheduling  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% gets list of plans, returns next action to perform %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% case 1 : first action of ongoing plan is feasible

% case 1a : physical action
% return first action of first plan - physical action
schedule_next_action(ListPlans,action(Agt,Name)) :-
        debug_message(schedule,['List of plans = ',ListPlans,' ']),
            instant(T),
            nth1(1,ListPlans,[ongoing,Plan]),
            nth1(1,Plan,action(Agt,Name)),
            debug_message(schedule,['---Scheduling : first action of first plan is a physical action ',action(Agt,Name)]),
            feasible(Agt,Name,T),!.

% case 1b: speech act
schedule_next_action(ListPlans,speechact(Force,Speaker,Hearer,CProp)) :-
            %instant(T),
            % get first plan
            nth1(1,ListPlans,[ongoing,Plan]),
            % get first element of plan AND sequel
            % nth1(1,Plan,speechact(Force,Speaker,Hearer,CProp)),
            Plan = [speechact(Force,Speaker,Hearer,CProp)|SequelOfPlan],
            debug_message(schedule,['---Scheduling : first action of first plan is a language action ',speechact(Force,Speaker,Hearer,CProp)]),
            % assert expectations for next user move (to compute surprise)
            what_expected(Speaker,SequelOfPlan),
            % check feasibility of this speech act
            debug_message(schedule,['---Scheduling : checking its feasibility... ']),
            sendable(speechact(Force,Speaker,Hearer,CProp)),!.

% FIXME with tag stored as first action, how is the plan updated when first action is performed?
            
% case 2: drop first plan if first action not feasible, try next plan
% if the 2 previous cases failed : drop first plan, recursive call
schedule_next_action(ListPlans,Action) :-
            debug_message(schedule,['---Scheduling : unfeasible plan dropped, switch to next plan, recursive scheduling...']),
            % mark ongoing plan as failed, promote next new plan as ongoing
            switch_plan(ListPlans,NewListPlans),
            % recursive call on updated list of plans
            schedule_next_action(NewListPlans,Action),!.
            
            
%            % get the agent main intention
%            main_intention(Agt,Prop),
%            remove_first_plan(Agt,Prop,FP),
%            perform_next_action(Agt,A),!.

% case 3 : no more new plans - display debug message and fail
schedule_next_action(_ListPlans,action(Agt,donothing,true)) :-
        current_agent(Agt),
        debug_message(schedule,['---Scheduling : all plans failed, no more plans to try: do nothing']),!.
     
            
% PB : remove the failed plan from the list in parameter, but not from the asserted my_plans ?
% if not then next time the first plan will be tried again and fail again
% instead of continuing the ongoing plan

% solution TODO assert which plan is ongoing, in addition to the list of computed plans
% predicate to switch_plan retracts ongoing plan and assert new ongoing plan
% predicate to assert which plans already failed
% so scheduling finds the ongoing plan to continue, or the next plan that hasnt failed yet
% FIXME use tags to name plans?


%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Assert expectations  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%

assert_expected(Agt,Qqun,Force) :- ifpossible(retract(expected(Agt,_,_)),assert(expected(Agt,Qqun,Force))).

what_expected(Agt,[SpeechAct|_Plan]) :- speaker(SpeechAct,Qqun),
					illoc_force(SpeechAct,Force),
					assert_expected(Agt,Qqun,Force),!.
										
what_expected(Agt,[action(Qqun,Action)|_Plan]) :- assert_expected(Agt,Qqun,Action),!.
										
what_expected(Agt,_) :- ifpossible(retract(expected(Agt,_,_)),true),!.				
									




%%%%%%%%%%%%%%%%
%% 3) ACTION  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Perform Next Action (external predicate) %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% perform the next action of the current plan by sending the corresponding action stimulus
% to be called only after planning(Agt) has computed and asserted the plans
% FIXME move action performance predicates to pleiad_action ?

perform_action(Action) :-
    % send action stimulus to current agent only - TODO: observers
    current_agent(Agt),
    stimulus(Agt,Action).



	
