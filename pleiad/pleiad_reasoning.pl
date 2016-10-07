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

% tag for debug: demo
% tag for debug undegrounded cci : grcci

debug_reasoning :-
    add_debug_tag(demo),add_debug_tag(grcci).

%%% TODO October 2012
% reasoning including panic (cf pleiad_perso.pl)
% excluding out of focus topics? or giving them less priority
% only take into account sufficiently activated mental attitudes (cf perso.pl)

%%% NEW November 2012
% T is now a parameter of all rules in pleiad_demo.pl to only consider current time
% Agt too: liit saturation to a single agent's KB
% rules have an index and can be inactivated

%% TODO May 2013
% specify index in active_rule
% so that in debug mode, deduce predicate can print the index of any rule applied




%%%%%%%%%%%%%%%%%%%%%%%%
%%  INFERENCE ENGINE  %%
%%%%%%%%%%%%%%%%%%%%%%%%
%% inference  engine  %%
%%%%%%%%%%%%%%%%%%%%%%%%

% compile demonstration rules (predicate rule\4)
:- [pleiad_demo].

% NEW November 2012
% active_rule to toggle rules on/off easily
% plus time instant as a parameter of rules to only use rules at current time
% plus agent as a parameter to only deduce things from one KB
active_rule_old(Agt,Prem,Conclu) :-
	active_rules(ListActiv),
	member(Index,ListActiv),
	instant(T),
	rule(Index,T,Agt,Prem,Conclu).
%% april 2013 : use all rules again
% or add their numbers to active_rules predicate on top of pleiad_demo.pl
active_rule(Index,Agt,Prem,Cci) :-
        instant(T),
        rule(Index,T,Agt,Prem,Cci).

% retro-compat abbrev: no specified agent = any agent
active_rule(Index,A,B) :- active_rule(Index,_Agt,A,B).

% new May 2013 : premises are now a list
% to check that the premises are valid, all members of the list need to be checked in order

% new July2014 - validity is for an agent : embeds predicates in beliefs to check them
% retrocompat
valid(L) :- current_agent(A),valid(A,L).
valid(A,L) :- is_list(L),valid_list(A,L).
valid(A,L) :- not(is_list(L)),valid_one(A,L).

% valid one : check one premise, to be called on all premises of a list
% mental attitudes checked directly
valid_one(Agt,MA) :- mental_attitude(Agt,MA),MA,!.

% existing predicate - checked directly. FIXME: declare all predicates?
valid_one(_Agt,Prop) :- functor(Prop,Predicate,Arity),current_predicate(Predicate/Arity),Prop.

% propositional contents of mental attitudes : embedded in a bel
% could be a mental attitude of another agent (need to check in his profile for this agent)
valid_one(Agt,Prop) :- not(mental_attitude(Agt,Prop)),bel(Agt,Prop,_,_,_),!.


% valid list : checks validity of all members of the list
valid_list(_Agt,[]) :- !.
valid_list(Agt,[P1|Prem]) :- valid_one(Agt,P1),
                    debug_message(valid,['Valid proposition for ',Agt,' : ',P1]),
                    valid_list(Agt,Prem).   %! final

% for debug purposes: display first invalid premises (FIXME how to display all but still fail in the end?)
valid_list(_Agt,[P1|_]) :- debug_message(valid,['!!! INvalid proposition : ',P1]),fail.

% deduce consequence using only active rules
% for a specified agent
deduce(Agt,B) :- active_rule(Index,Agt,A,B),debug_message(valid,['checking validity of rule ',Index]),valid(A),  % not(B)
                 if_debug_tag(demo,((not(B),print('apply rule '+Index+' for agent '+Agt+' to deduce '+B+' from '+A+'\n'));true)).   

% for debug : impose index of rule to be used
deduce(Agt,Index,B) :- active_rule(Index,Agt,A,B),valid(A).

% order list of indexes of active rules - debug
list_active_rules_index(Agt,LL) :-
    findall(I,active_rule(I,Agt,_,_),LIndex),list_to_set(LIndex,SIndex),sort(SIndex,LL).


% for any agent (retro-compat abbrev)
deduce(B) :- deduce(_Agt,B).

% for debug
% add_debug_tag(demo) to get traces
deduce_debug(Agt,Index,B) :- instant(T),rule(Index,T,Agt,A,B),valid(A),if_debug_tag(demo,display_message(['valid rule ',Index,' deduces ',B,' from ',A,'\n\n'])).
% writeln('valid rule deduces '+B+' from '+A)

deduce_debug(Agt,Index,B) :- instant(T),rule(Index,T,Agt,A,B),not(valid(A)),if_debug_tag(demo,writeln('invalid premise '+A)).

deduce_debug(Agt,Index,_B) :- instant(T),not(rule(Index,T,Agt,_,_)),if_debug_tag(demo,writeln('no valid rule with index '+Index)).
                                                                               
% debug agent
% list_active_rules_index(Agt,LL),member(Index,LL),deduce_debug(coco,Index,B).


% debug_demo_rules without params
debug_demo_rules :-
    % current agent
    current_agent(A),
    % no constraints on Index
    debug_demo_rules(A,_).

% debug demo rules for an agent
debug_demo_rules(Agt,IndexFail) :-
    % list of indexes of rules to try
    list_active_rules_index(Agt,LL),
    debug_demo_rules_list(Agt,LL,IndexFail).

debug_demo_rules_list(Agt,[I|L],IndexFail) :-
        instant(T),rule(I,T,Agt,Prem,_Cci),
        display_message(['checking rule ',I,' with premise ',Prem,' ']),
        valid(Prem),writeln('rule '+I+' OK ! applicable'),
        debug_demo_rules_list(Agt,L,IndexFail),!.

debug_demo_rules_list(Agt,[I|L],IndexFail) :-
        instant(T),rule(I,T,Agt,Prem,_Cci),not(valid(Prem)),writeln('rule '+I+' OK ! not applicable'),
        debug_demo_rules_list(Agt,L,IndexFail),!.

debug_demo_rules_list(_Agt,[IndexFail|_L],IndexFail) :-
        writeln('rule '+IndexFail+' FAILED !'),!.
        % , debug_demo_rules_list(Agt,L).



%%%%
%% debugging not grounded conclusions
%%%%

% with debug for ungrounded cci
debug_ungrounded_cci(Agt,ListUC) :-
    findall([Index,B],deduce_debug(Agt,Index,B),ListIB),
    findall([IndexU,BU],(member([IndexU,BU],ListIB),not(ground(BU))),ListUC).

% to find ungrounded intentions : debug_ungrounded_cci(coco,L),member([Index,intend(Agt,Prop,Deg,Time,Src)],L).







%%%%%%%%%%%%
%% saturate BC once
%% ie make all deductions once
%% warning: needs to be done until nothing more can be deduced, using saturer_bc
%%%%%%%%%%%%

% saturate KB = make all possible deductions (one time)
% in one KB of a single agent
saturer_bc_once(Agt) :- 
	findall(B,deduce(Agt,B),P),
	forall((member(A,P),not(A)),s_assert(A)).


% IDEA Nov2012: load_kb asserts the current agent to be the loaded one?
% or adds the agent to the list of loaded agents?
% also needs a reinit predicate that removes agent from list of loaded agents

%%%%%%%%
%% saturation loop : saturate KB as long as sth new can be deduced
%%%%%%%%

% NEW November 2012 
%% saturate only KB of one specific agent

% findall mental attitudes, then saturate, then refindall, 
% then find differences between the 2 lists
% and if sth new appears then make a recursive call
% vecteur_beliefs, vecteur_amis, vecteur_desires (pleiad_tools)
% current demo rules only produce desires and bels 
% (to be completed later with other vectors)

saturer_bc_aux(A) :- 
	%% NEW optimisation: only consider current time T
	% instant(T),
	%% list all mental attitudes at time T before update
	% vecteur_belief(A,T,LB1),
	% vecteur_desires(A,LD1),
	% merge(LB1,LD1,L1),
        % new june 2013: vecteur of all BDI at current instant T
        vecteur_bdi(Agt,L1),
	%% update KB once
	saturer_bc_once(A),
	%% list all mental attitudes again after this update
	% vecteur_belief(A,T,LB2),vecteur_desires(A,LD2),merge(LB2,LD2,L2),
        vecteur_bdi(Agt,L2),
	%% search differences 
	findall(Z,(member(Z,L2),not(member(Z,L1))),ZZ),
        if_debug_tag(satur,forall(member(Z,ZZ),display_message(['Saturate KB added new mental attitude : ',Z,' ']))),
	%% if list of differences is not empty: recursive call; else stop here
	((member(_,ZZ),saturer_bc(A));true).

% with a cut (for some reason)
% new june 2014 - also use Jeremy specific intention rules (pleiad_intentions.pl)
saturer_bc(A) :- saturer_bc_aux(A),!.

% NEW JULY 2014 - saturate KB + return list of new mental attitudes
% from Jeremy - used only in emotions_j/event_evaluation
% rule_csq exported as standard demo rules, removed deduce_consequences and saturer_csq that only used rule_csq,
% replaced with following predicate saturer_bc_return that uses all rules AND returns list of new mental attitudes
% TODO compare mental attitudes between consecutive time instants

saturer_bc_aux_return(A,NewCsq) :- 
	% new june 2013: vecteur of all BDI at current instant T
        vecteur_bdi(Agt,L1),
	%% update KB once
	saturer_bc_once(A),
	%% list all mental attitudes again after this update
	% vecteur_belief(A,T,LB2),vecteur_desires(A,LD2),merge(LB2,LD2,L2),
        vecteur_bdi(Agt,L2),
	%% search differences 
	findall(Z,(member(Z,L2),not(member(Z,L1))),ZZ),
        if_debug_tag(satur,forall(member(Z,ZZ),display_message(['Saturate KB added new mental attitude : ',Z,' ']))),
	%% if list of differences is not empty: recursive call; else stop here
	((member(_,ZZ),saturer_bc_return(A,MoreCsq),merge(ZZ,MoreCsq,NewCsq));(true,NewCsq=ZZ)).

saturer_bc_return(A,NewCsq) :- saturer_bc_aux_return(A,NewCsq),!.

% debug call : immediately retract all new mental attitudes - debug only
% saturer_bc_return(toto,L),forall(member(LL,L),retract(LL)).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MERGING DEGREES of mental attitudes  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% to be used to compute final degree in demo rules
% to avoid problems in case one degree is not instantiated
% and to make it easier to change the merge function

merge_degs(DegreeList,Degree) :-
    findall(Deg,(member(Deg,DegreeList),ground(Deg)),InstDegList),
    % length(InstDegList,Nb),
    merge_inst_degs(InstDegList,Degree),!.

% in case no degree is instantiated
merge_inst_degs([],0) :- writeln('ERROR: no instantiated degree in the list to merge! returning degree=0'),!.
% if there is at least one instantiated degree in the list
merge_inst_degs([Deg],Deg) :- !.
% more than 1 degree: recursive call
merge_inst_degs([Deg|List],MDeg) :-
    % merge the rest of the list into one degree
    merge_inst_degs(List,Deg2),
    % merge it with first degree
    MDeg is Deg * Deg2.




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ASSERTIONS OF CONCLUSIONS %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% asserting consequences of demo rules %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% OLD %%%
% special treatment when asserting beliefs 
% (to handle conflicts with previously existing beliefs)
%s_assert(believe(Agt,Prop,Deg,Tps)) :- assert_belief(Agt,Prop,Deg,Tps,deduction).
%s_assert(P) :- P \= believe(_,_,_,_,_),assert(P).

%%% NEW %%%
%% asserting beliefs (handling conflicts with pre-existing beliefs)
% and replacing BelTime with current instant (only assert beliefs at present time)
s_assert(bel(Agt,Prop,Deg,_Tps,Canal)) :- 
	instant(T), assert_belief(Agt,Prop,Deg,T,Canal).

%% asserting new intentions
s_assert(intend(Agt,Prop,Deg,Tps,Src)) :- 
	% print('New intend: '+ Prop+Deg+Src+'\n'), MOVED to assert_intention
	% this specific predicate manages conflicts (retracts previous intention on same Prop)
	assert_intention(Agt,Prop,Deg,Tps,Src).

%% asserting new goals - no goals in my version October 2012
% NEW MAY 2013 added goal(Agt,Prop,Deg,Time,Src)
s_assert(goal(Agt,Prop,Deg,Time,Src)) :- assert_goal(Agt,Prop,Deg,Time,Src).
s_assert(notgoal(Agt,Prop,Deg,Time,Src)) :- assert_notgoal(Agt,Prop,Deg,Time,Src).

% asserting new groundings
s_assert(grounded(Group,Prop)) :- assert_grounding(Group,Prop).

% default case
s_assert(P) :- 
	P \= bel(_,_,_,_,_), 
	P \= intend(_,_,_,_,_), 
        P \= grounded(_,_),
	P \= goal(_,_,_,_,_), 
	P \= notgoal(_,_,_,_,_), 
	assert(P).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% PHASES : perception, decision, action  %%
%% influence which rules can be used in   %%
%% agent reasoning                        %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% default phase = perception
% nextphase predicate unused

nextphase(Agt) :-
    retract(phase(Agt,perception)), assert(phase(Agt,decision)),!.
    
nextphase(Agt) :-
    retract(phase(Agt,decision)), assert(phase(Agt,action)),!.
    
nextphase(Agt) :-
    retract(phase(Agt,action)), assert(phase(Agt,perception)),!.


%%%%
%% changing phase with no backtrack and no fail
%%%%

reset_phase(Agt) :- retract(phase(Agt,P)), assert(phase(Agt,perception)), debug_message(loop,['Agent ',Agt,' was in phase ',P,', now back to perception ']),!.
reset_phase(Agt) :- not(phase(Agt,_)), assert(phase(Agt,perception)), debug_message(loop,['Agent ',Agt,' was in NO phase, now in perception ']),!.

enter_perception_phase(Agt) :- phase(Agt,perception),!.
enter_perception_phase(Agt) :- retract(phase(Agt,_Action)),assert(phase(Agt,perception)),!.
%enter_perception_phase(Agt) :- phase(Agt,P),writeln('agent '+Agt+' cannot enter perception phase, current phase '+P).

enter_decision_phase(Agt) :- phase(Agt,decision),!.
enter_decision_phase(Agt) :- retract(phase(Agt,_Perception)),assert(phase(Agt,decision)),!.
%enter_decision_phase(Agt) :- phase(Agt,P),writeln('agent '+Agt+' cannot enter decision phase, current phase '+P).

enter_planning_phase(Agt) :- phase(Agt,planning),!.
enter_planning_phase(Agt) :- retract(phase(Agt,_Decision)),assert(phase(Agt,planning)),!.
%enter_planning_phase(Agt) :- phase(Agt,P),writeln('agent '+Agt+' cannot enter planning phase, current phase '+P).

enter_schedule_phase(Agt) :- phase(Agt,schedule),!.
enter_schedule_phase(Agt) :- retract(phase(Agt,_Planning)),assert(phase(Agt,schedule)),!.
%enter_schedule_phase(Agt) :- phase(Agt,P),writeln('agent '+Agt+' cannot enter schedule phase, current phase '+P).

enter_action_phase(Agt) :- phase(Agt,action),!.
enter_action_phase(Agt) :- retract(phase(Agt,_Schedule)),assert(phase(Agt,action)),!.
%enter_action_phase(Agt) :- phase(Agt,P),writeln('agent '+Agt+' cannot enter action phase, current phase '+P).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% PROACTIVE AND REACTIVE         %%
%% REASONING CYCLE                %%
%% PERCEPTION - DECISION - ACTION %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 1) perception predicate : cf pleiad_stimulig.pl - does the KB saturation

% 2) decision phase = deduce one intention from goals, then saturate to plan to reach it

% return action that the agent decides to perform
% new june2014 or the proposition the agent decides to reach - also new source of intention can be anything, not just decision
% new june2014 removed degint
deliberate(Agt,Agenda) :-   % IntendedProp
    % current instant
    % instant(T),
    % go out of perception phase, to decision phase
    enter_decision_phase(Agt),
    % decide of which intention to pursue and how to reach it (once in decision phase, different rules apply)
    saturer_bc(Agt),
    % agent's intention to be returned by the predicate
    % intend(Agt,IntendedProp,DegInt,T,_AnySrc).   % done(Agt,IntendedAction,_Sometime)
    % should not just be the first intention found!
    % compute agenda of all intentions
    compute_intention_agenda(Agt),
    % then just read main intention in agenda - fails if agenda exhausted
    % main_intention(Agt,IntendedProp).
    % new : returns full agenda instead of main intention
    agenda(Agt,Agenda).

% delib should NOT fail - happens if no intention, hence no agenda...   
deliberate(Agt,bug) :-
    debug_message(loop,['Agent ',Agt,' deliberation failed : BUG !!!']),!.
    
% 3) planning phase

% plan : do planning loop, them initialise final plans and asserted them
% IN: agent and agenda as computed by deliberation
% OUT: initialised plans, also asserted with predicate my_plans
% plan() needs to assert initialised plans for scheduling to work without re-computing / re-initialising
plan(Agent,Agenda,MyProp,MyPlansI) :-
    enter_planning_phase(Agent),
    %plan_until_good(Agent,Agenda,MyProp,MyPlansNI),!,
    %debug_message(loop,['---Plan loop : end of loop, computed plans = ',MyPlansNI,' for reaching ',MyProp,' - initialising...']),
    plan_all_agenda(Agent,Agenda,MyProp,MyPlansNI),!,
    debug_message(loop,['---Plan all agenda : over, first computed plans = ',MyPlansNI,' for reaching main intention ',MyProp,' - initialising...']),
    % initialise all plans and re-assert them (only if not already asserted)
    initialise_asserted_plans(Agent),
    % get plans for main intention (for debug display + return)
    my_plans(Agent,MyProp,MyPlansI),
    debug_message(loop,['---Plan all agenda : initialised plans = ',MyPlansI,' - asserting...']).
    

% 4) schedule action phase

schedule(Agt,ListInitPlans,NextAction) :-
    enter_schedule_phase(Agt),
    schedule_next_action(ListInitPlans,NextAction).
    
% 5) action performance phase

act(Agt,Action) :-
    % instant(T),
    % get out of decision phase - to action phase
    % ((retract(phase(Agt,decision)),assert(phase(Agt,action)));phase(Agt,action)),
    enter_action_phase(Agt),
    perform_action(Action),
    % actually perform action, send stimulus : cf pleiad_actions.pl
    %perform(Agt,Action,[]), FIXME which perform predicate?
    saturer_bc(Agt).


%%%%%%%%%%%%%%%%%%%%%%%%
%% LOOP - LOOP - LOOP %%
%%%%%%%%%%%%%%%%%%%%%%%%

% main loop when all goes well    
agent_main_loop(Agent,Action) :- 
    deliberate(Agent,Agenda),!,
    debug_message(loop,['*** DELIBERATION phase : ',Agent,' computed agenda ',Agenda,' , starting planning...']),
    % new JULY2014 - plan in a while loop until plans not empty
    plan(Agent,Agenda,MyProp,MyPlansI),
    debug_message(loop,['*** PLANNING phase : ',Agent,' computed list of (initialised) plans ',MyPlansI,' for goal ',MyProp,', starting scheduling...']),
    schedule(Agent,MyPlansI,Action),!,
    debug_message(loop,['*** SCHEDULING phase : next action is ',Action,' , starting performance...']),
    act(Agent,Action),!,
    debug_message(loop,['*** ACTION phase ',Action,' successfully performed, now moving to next time step...']),
    avancer(Agent),!,
    debug_message(loop,['*** Moved to next time step - end of loop\n ******* \n ']),
    !.

agent_reactive_loop(Agent,Stimulus,Action) :-
    debug_message(loop,['*******\n Starting reactive loop of agent ',Agent,' ******* ']),
    %debug_message(loop,['*** Starting perception...']),
    perceive(Agent,Stimulus),!,
    %saturer_bc(Agent), % done in perceive
    %avancer
    debug_message(loop,['*** PERCEPTION phase : ',Agent,' perceived stimulus ',Stimulus,' , starting deliberation...']),
    % FIXME should separate perception and KB saturation
    agent_main_loop(Agent,Action).
    
agent_proactive_loop(Agent,Action) :-
    debug_message(loop,['*******\n Starting proactive loop of agent ',Agent,' ']),
    saturer_bc(Agent),!,
    debug_message(loop,['*** DEDUCTION phase : KB saturated, starting deliberation...']),
    agent_main_loop(Agent,Action).


