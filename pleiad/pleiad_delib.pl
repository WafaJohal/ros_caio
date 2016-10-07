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


% file content
% 1) goal_pref_src to find goal from preferred source
% 2) demo rule to deduce intention from preferred source goal
% 3) main_intention to get agent's main intention to be sent to planning

:- assert(agenda(test,[])).

%%%%%%%%%%%%%%%%%%%%%%
%% 1) goal pref src %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% find goal from preferred source  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% NEW June 2013: find goal from preferred source, and then highest degree
%% (take into account source of goal, personality for order of preference, and priority)
% different cases are mutually exclusive

% goal_pref_src(Agt,goal(Agt,P,D,T,S1)) :-
%    prefgoal(Agt,[S1,_S2,_S3]),
%    goal(Agt,P,D,T,S1),!.

% goal_pref_src(Agt,goal(Agt,P,D,T,S2)) :-
%    prefgoal(Agt,[_S1,S2,_S3]),
%    goal(Agt,P,D,T,S2),!.

% goal_pref_src(Agt,goal(Agt,P,D,T,S3)) :-
%    prefgoal(Agt,[_S1,_S2,S3]),
%    goal(Agt,P,D,T,S3),!.

% problem 1: several goals of same source? will create several intentions (rule 10 in pleiad_demo)
% problem 2: goal to perform an illoc act + goal to utter corresponding words 

% new July 2013: only consider goals that are not achieved yet
% otherwise the preferred goal could be already achieved so no intention deduced and no action
% while there was a less preferred source leading to a still achievable goal and thus intention thus action

% returns the list of all goals in order
goals_pref_src(Agt,List) :-
    % only consider current goals
    instant(T),
    % order of preference for goal sources
    prefgoal(Agt,[S1,S2,S3]),

    % find all unachieved goals of source S1
    findall(goal(Agt,P,D,T,S1),(goal(Agt,P,D,T,S1),not(bel(Agt,P,_,_,_))),ListGoalS1),
    % sort by decreasing degrees of priority
    sort_goal(ListGoalS1,SortedLGS1),

    % of source S2
    findall(goal(Agt,P,D,T,S2),(goal(Agt,P,D,T,S2),not(bel(Agt,P,_,_,_))),ListGoalS2),
    sort_goal(ListGoalS2,SortedLGS2),
    
    % then of source S3
    findall(goal(Agt,P,D,T,S3),(goal(Agt,P,D,T,S3),not(bel(Agt,P,_,_,_))),ListGoalS3),
    sort_goal(ListGoalS3,SortedLGS3),
    
    % append three sources in order (do not use merge, does not keep order!)
    append(SortedLGS1,SortedLGS2,Aux),
    append(Aux,SortedLGS3,List),!.

% preferred goal = head of this list (could be a goal to achieve a proposition)
goal_pref_src(Agt,Goal) :- goals_pref_src(Agt,[Goal|_]).

% preferred goal to perform an action (and not to achieve a proposition)
% as we do NOT want intentions that are not about actions
actiongoal_pref_src(Agt,Goal) :- goals_pref_src(Agt,List),find_first_actiongoal(List,Goal).

% recursive auxiliary function to find the first action goal in the list
% if the first goal in the list is an action goal, that's the one
find_first_actiongoal([goal(Agt,done(Agt,Action,Time),DegGoal,TimeGoal,SrcGoal)|_],
                      goal(Agt,done(Agt,Action,Time),DegGoal,TimeGoal,SrcGoal)) :- !. 
% if not, recursive call
find_first_actiongoal([_|List],Goal) :- find_first_actiongoal(List,Goal),!.
% this will fail if there is no action goal in the list (which is the wanted behaviour, and situation should not happen hopefully)





%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2) GOAL TO INTENTION %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% from goals to intentions - demo rules  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% FIXME: if the goal_pref_src is already achieved, then no intention is deduced...
%% but that should not happen...

% first rule: there is a goal from first preferred source
% TODO : this is a rule for decision phase
% rule 10a : goal to achieve a proposition that is not done(action)

% agents can NOT have intention to reach a proposition -
% FIXME june2014 yes they can? finding the action to reach a propgoal should be part of planning?
% prules work on goals, then when all goals are deduced,
% the preferred ACTION goal is turned into the agent's intention
% rule(101,T,Agt,
% 	% preconds
%	    ([
%            % order of goal sources (in personality of agent)
%            % used to find preferred goal from preferred source
%            goal_pref_src(Agt,goal(Agt,P,D,T,_S)),
%            % as extra-safety: check P was not achieved yet
%            not(believe(Agt,P,_Anydeg,_Anytime)),
%            % P is not the performing of an action
%            P \= done(_,_,_),
%            % no intention yet at this instant from any source
%            not(intend(Agt,_,_,T,_)),
%            % and it is decision phase
%            phase(Agt,decision)
%            ]),
%	% csqs: adopt this goal as intention
%	intend(Agt,P,D,T,planif)
% ).


% rule 10b : goal to perform an action (that should not be a speech act)
% rule(102,T,Agt,
%	% preconds
%	    ([
%            % order of goal sources (in personality of agent)
%            % used to find preferred goal (cf pleiad_planif.pl)
%            actiongoal_pref_src(Agt,goal(Agt,done(Agt,A,TD),D,T,_SG)),
%            % P is to perform an action that is not a speech act
%            not(speech_act(A,_B)),
%            % action not done yet before
%            not(bel(Agt,done(Agt,A,_),_Anydeg,_Anytime,_Anysrc)),
%            % no other intention yet at this instant
%            not(intend(Agt,_,_,T,_SI)),
%            % and it is decision phase
%            phase(Agt,decision)
%            ]),
%	% csqs: adopt this goal as intention
%	intend(Agt,done(Agt,A,TD),D,T,planif)
% ).

% new july2014 - commented rules 101 and 102 - assert intention for all goals but with priority
% so that the deliberation process can build a complete agenda
rule(101,T,Agt,
    % premises
    ([
        % find all goals in order of priority
        goals_pref_src(Agt,ListGoals),
        % get index to ponderate priority
        nth0(Index,ListGoals,goal(Agt,Prop,Deg,Time,_Src)),
        % not already an intention for this prop
        not(intend(Agt,Prop,_AnyDeg,Time,_AnySrc)),
        % compute new priority of intention from position in list of goals
        Prio is max(0,(1-0.1*Index)*Deg)
    ]),
    % conclusion
    intend(Agt,Prop,Prio,T,planif)
).

            
% rule 10c : goal to perform a speech act creates intention to utter the corresponding locutionary act
%           and NOT the intention to perform the illocutionary act

rule(103,T,Agt,
	% preconds
	    ([
            % order of goal sources (in personality of agent)
            % used to find preferred goal (cf pleiad_planif.pl)
            actiongoal_pref_src(Agt,goal(Agt,done(Agt,IllocAct,TD),D,T,_SG)),
            % speech act not done yet
            hearer(IllocAct,B),
            not(grounded([Agt,B],done(Agt,IllocAct,_))),
            % P is to perform an action that is not a speech act
            speech_act(IllocAct,LocAct),
            % locutionary act was not performed yet
            not(bel(Agt,done(Agt,LocAct,_),_Anydeg,_Anytime,_Anysrc)),
            % no intention yet at this instant
            not(intend(Agt,_,_,T,_SI)),
            % and it is decision phase
            phase(Agt,decision)
            ]),
	% csqs: adopt this goal as intention
	intend(Agt,done(Agt,LocAct,TD),D,T,planif)
).



%%%%%%%%%%%%%%%%%%%%%%%%
%% 3) MAIN INTENTION  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Get the main intention by Source %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Find the intend with the higher degree
% Fevrier - not used anymore?
%max_intend_degree([intend(Agt,P,D,T,Src),intend(Agt,_P1,D1,T,_Src1)|List],intend(Agt,P2,D2,T,Src2)) :- D >= D1, max_intend_degree([intend(Agt,P,D,T,Src)|List],intend(Agt,P2,D2,T,Src2)).
%max_intend_degree([intend(Agt,_P,D,T,_Src),intend(Agt,P1,D1,T,Src1)|List],intend(Agt,P2,D2,T,Src2)) :- D < D1, max_intend_degree([intend(Agt,P1,D1,T,Src1)|List],intend(Agt,P2,D2,T,Src2)).
%max_intend_degree([intend(Agt,P,D,T,Src)],intend(Agt,P,D,T,Src)).

% intention_by_src_aux(Agt,P,Src) :-
%			instant(T),
%			vecteur_intend(Agt,T,Intend_List),
%			findall(intend(Agt,Prop,D,T,Src),member(intend(Agt,Prop,D,T,Src),Intend_List),List_Src),
%			max_intend_degree(List_Src,intend(Agt,P,_Deg,T,Src)).
	
% 3 levels of priority for intentions : first emotion, then dialogue, then global	
%main_intention(Agt,Prop) :- intention_by_src_aux(Agt,Prop,localEmotion),!.
%main_intention(Agt,Prop) :- intention_by_src_aux(Agt,Prop,localDialogue),!.			
%main_intention(Agt,Prop) :- intention_by_src_aux(Agt,Prop,global).			

        
%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ACTUAL DELIBERATION  %%
%% USING AGENDA         %%
%%%%%%%%%%%%%%%%%%%%%%%%%%

% TODO : optimise by splitting into
% 1) compute agenda (only once) - or update it if needed
% 2) get main intention : only reads main tag in agenda, does NOT compute or change anything

main_intention(Agt,Prop) :-
                            agenda(Agt,Agenda),
                            member([main,intend(Agt,Prop,_,_,_)],Agenda),!.


% case 1a : agenda built, main intention has failed, update agenda
% compute_intention_agenda(Agt,Prop) :- %instant(T),
%                            agenda(Agt,Agenda),
%                            member([main,intend(Agt,OldProp,_,_,_)],Agenda),
%                            my_plans(Agt,OldProp,[]),
%                            update_agenda(Agt),
%                            member([main,intend(Agt,Prop,_,_,_)],Agenda),!.

% case 1: agenda already built
% main intention is now taken from agenda
compute_intention_agenda(Agt) :- %instant(T),
                            agenda(Agt,_Agenda),
                            % update (only if needed)
                            update_agenda(Agt),!.

% case 2 : no agenda built yet
compute_intention_agenda(Agt) :- %instant(T),
                            not(agenda(Agt,_)),
                            assert_agenda(Agt),
                            %agenda(Agt,Agenda),
                            !.
                            
% case 3 : agenda is exhausted
% compute_intention_agenda(Agt,done(Agt,noaction,_)) :-
%                    %instant(T),
%                    agenda(Agt,Agd),
%                    not(member([main,_],Agd)),!.
             
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 4) AGENDA of intentions  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% NEW JUne2014 - replaces main intention 
% return a sorted list of intentions called agenda, assert it with tags : ongoing, next, failed
% when agenda is empty agent tries backup plan, eg alice, or random action (smalltalk)
% pb : how to interleave intentions from different sources and different priorities? ponderate degrees?
% only put top intention of each source in the agenda ? then the rest in priority order

assert_agenda(Agt) :-
                instant(T),
                % find all intentions
                vecteur_intend(Agt,T,Intend_List),
                % call auxiliary predicate (so far agenda is empty)
                build_agenda_aux(Intend_List,1,AgendaIntentions),
                initialise_agenda(AgendaIntentions,AgendaInit),
                % assert agenda as computed
                nofail(retractall(agenda(Agt,_))),
                assert(agenda(Agt,AgendaInit)),!.
                
                % split into 3 lists by source - but what if one list is empty ?  
%                findall(intend(Agt,Prop,D,T,localEmotion),member(intend(Agt,Prop,D,T,localEmotion),Intend_List),[EmoIntend|EmoList]),
%                findall(intend(Agt,Prop,D,T,localDialogue),member(intend(Agt,Prop,D,T,localDialogue),Intend_List),[LocalIntend|LocalList]),
%                findall(intend(Agt,Prop,D,T,global),member(intend(Agt,Prop,D,T,global),Intend_List),[GlobalIntend|GlobalList]),
  

  % backup case if no intention at this instant for this agent: assert empty agenda
assert_agenda(Agt) :-
        % no intention
        instant(T),
        vecteur_intend(Agt,T,[]),
        nofail(retractall(agenda(Agt,_))),
        assert(agenda(Agt,[])),!.
        
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% BUILD agenda from list of intentions %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
% case 1 : there is an emotion intention  
build_agenda_aux(ListIntend,1,[intend(Agt,Prop,D,T,localEmotion)|Agenda]) :-
            % remove first emo intention from list of all intentions, add it to agenda - cut to get only first one
            % FIXME how to check there is no emotion intention in the agenda yet?
            select(intend(Agt,Prop,D,T,localEmotion),ListIntend,RemainingIntend),!,
            % recursive call to build the rest of the agenda (what follows that first intention)
            build_agenda_aux(RemainingIntend,2,Agenda),!.

% if no emotion intention at all
build_agenda_aux(ListIntend,1,Agenda) :-
            	% no emotion intention in the list of intentions
		not(member(intend(_Agt,_Prop,_D,_T,localEmotion),ListIntend)), 
		build_agenda_aux(ListIntend,2,Agenda),!.
            
% case 2 : there is a local dialogue intention
build_agenda_aux(ListIntend,2,[intend(Agt,Prop,D,T,localDialogue)|Agenda]) :-
            % remove first local dialogue intention from list of all intentions, add it to agenda - cut to get only first one
            select(intend(Agt,Prop,D,T,localDialogue),ListIntend,RemainingIntend),!,
            % recursive call
            build_agenda_aux(RemainingIntend,3,Agenda),!.

% if no emotion intention at all
build_agenda_aux(ListIntend,2,Agenda) :-
            	not(member(intend(_Agt,_Prop,_D,_T,localDialogue),ListIntend)), 
		build_agenda_aux(ListIntend,3,Agenda),!.

% case 3 : there is a global dialogue intention
build_agenda_aux(ListIntend,3,[intend(Agt,Prop,D,T,globalDialogue)|Agenda]) :-
            % remove first global dialogue intention from list of all intentions, add it to agenda - cut to get only first one
            select(intend(Agt,Prop,D,T,globalDialogue),ListIntend,RemainingIntend),!,
            % recursive call
            build_agenda_aux(RemainingIntend,4,Agenda),!.

% if no emotion intention at all
build_agenda_aux(ListIntend,3,Agenda) :-
            not(member(intend(_Agt,_Prop,_D,_T,globalDialogue),ListIntend)), build_agenda_aux(ListIntend,4,Agenda),!.
            
% case 4 : put all other intentions in priority order
% also captures the case when there is no intention at all (which is annoying)
build_agenda_aux(ListIntend,4,ListIntend) :- !.
    
%%%%%%%%%%%%%%%%%%%%%%%%
%% INITIALISE agenda  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% first intention is main, all other intentions are next %%
%% failed intentions will be marked as such when failing  %%
%% similar to initialise & switch plans in scheduling.pl  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% base case for init : if empty, initialisation does nothing
initialise_agenda([],[]) :- !.

% normal case: agenda is not empty
initialise_agenda([FirstIntention|ListIntentions],[[main,FirstIntention]|ListInitIntentions]) :-
        % append tag main at the start of first intention
        % append tag next at the start of all other plans (except first one)
        findall([next,Intention],member(Intention,ListIntentions),ListInitIntentions).
              
%%%%%%%%%%%%%%%%%%%%
%% UPDATE agenda  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% mark intention as failed, switch to next one %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

update_agenda(Agt) :-
        %instant(T),
        debug_message(delib,['---Update : agent ',Agt,' checks if agenda should be updated']),
        agenda(Agt,OldListIntend),
        member([main,intend(Agt,Prop,_Deg,_Time,_Src)],OldListIntend),
        debug_message(delib,['---Update : main intention is to reach proposition ',Prop,' -- getting computed plans']),
        my_plans(Agt,Prop,[]),
        debug_message(delib,['---Update : computed plans for this intention are empty ! update needed']),
        % switch intention first, in case it fails
        switch_intention(OldListIntend,NewListIntend),
        % then retract agenda only if switch succeeded
        % warning if no agenda to retract, OldList not instantiated, retract fails, retractall succeeds
        retract(agenda(Agt,OldListIntend)),!,
        assert(agenda(Agt,NewListIntend)).
       
update_agenda(Agt) :-
        % if no reason to update, do nothing
        debug_message(delib,['---Update : update agenda of agent ',Agt,' does nothing']),
        !.
       
% mark ongoing plan as failed, then find next plan still marked as new and make it ongoing
% prints failed plan (for debug and self-explaining purposes)
switch_intention(ListIntentions,NewListIntentions) :-
        % ground(ListIntentions), - NO or it fails because of partial instantiation and gets to bug case 2 NOt exhausted...
        % remove ongoing Intention from the list of Intentions
        debug_message(delib,['---Switch : switching intention in agenda ',ListIntentions,' ']),
        select([main|OngoingIntention],ListIntentions,TmpList1),
        % replace main tag with failed tag and insert failed Intention in the tmp list
        nth0(0,TmpList2,[failed|OngoingIntention],TmpList1),
        debug_message(delib,['---Switch : dropped failed intention ',OngoingIntention,' ']),
        % find first next Intention and remove it from tmp list
        select([next|NewIntention],TmpList2,TmpList3),
        % make it main (fails if no more next intention), insert it back into tmp list
        nth0(0,NewListIntentions,[main|NewIntention],TmpList3),
        debug_message(delib,['---Switch : new agenda ',NewListIntentions,' ']),
        debug_message(delib,['---Switch : switched to next new intention ',NewIntention,' ']),!.

% if the failed intention was the last one : do not fail, keep the same agenda
switch_intention(ListIntentions,TmpList2) :-
        % extract main intention
        select([main|OngoingIntention],ListIntentions,TmpList1),
        % replace main tag with failed tag and insert failed Intention in the tmp list
        nth0(0,TmpList2,[failed|OngoingIntention],TmpList1),
        % check that only failed intentions are in the agenda
        %not((member([Test,_],ListIntentions),Test\=failed)), % out of global stack
        debug_message(delib,['---Switch : checking if agenda is exhausted : ',TmpList2,' ']),
        % FIXME agenda should be exhausted here since first case of switch_intention failed to find a new intention in agenda...
        ((exhausted(TmpList2),
        debug_message(delib,['---Switch : YES, it was the LAST failed intention, exhausted agenda not modified ',TmpList2,' ']));
         debug_message(delib,['---Switch : NO, not exhausted, do nothing'])),!.


% base case (empty agenda, should not happen)
exhausted([]) :- !.
% all elements should be failed. Predicate exhausted() fails at the first non-failed intention found in agenda
exhausted([[failed,_]|Agenda]) :- exhausted(Agenda).


