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

% tag for debug : propag

%% IDEA October 2012
% have a pleiad_degrees.pl file
% that handles all operations on degrees (be it focus or other, if there is any other)

% new june2014 - avancer from Jeremy time_forward.pl that handles intention and plans propagation and update


%%%%%%%%%%%%%%%%%%%%%%%%%%
%% TIME MANAGING MODULE %%
%%%%%%%%%%%%%%%%%%%%%%%%%%

%% handles
%   - emotional persistance
%   - time forward
%   - focus management (decay)
%   - belief propagation
%   - goal propagation (new june 2013)


reset_time :- retract(instant(_)),assert(instant(0)).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% emotional persistance    %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 1) to be done with current instant before moving time forward
% 2) store felt emotions at that instant because they will not be deducible anymore at the next instant
% (because focus at past instants is not stored) and they are needed to reason (eg about confirmation emotions)
%
store_emotion(T) :- forall(emotion(A,E,P,D,T),kassert(emotion(A,E,P,D,T))).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% gestion du temps %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% init: when loading pleiad initially, assert start instant to be 0
%% :- assert(instant(0)).  --> moved to pleiad.pl


%% move time forward
%
avancer(Agt) :- 
	% get previous value of current instant
	instant(T),
	% compute next instant
	T1 is T+1, 
        % print out warning
        writeln('>>> Move time forward from '+T+' to '+T1),
        % new July: saturate KB to make sure the agent knows about goals that were just achieved
        saturer_bc(Agt),
	% DO NOT (new april 2013) retract previous value of last stimulus (without failing if no last stimulus asserted)
	%APRIL2013% nofail(retract(laststimulus(_,_))),
	% store emotions that were triggered at time T
	store_emotion(T),
	% replace previous instant with new one
	retract(instant(T)),assert(instant(T1)),
	% decay focus over time
	manage_focus(T1),
	% propagate previous beliefs to the next instant
	maintain_belief(T1),
        % clean old beliefs
        retract_old_beliefs(Agt,T1),
        % NEW JUNE 2013 : propagate goals
        maintain_goal(T1),
        maintain_condgoal(T1),
        % new june 2014 : propagate intentions
        propag_intentions(T1),
        % and update plans - FIXME : needs to saturate KB frst?
        % FIXME - to be done in planning phase rather than avancer?
        update_all_my_plans(Agt).  %,planning(Agt).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% gestion du focus %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%
% focus decay management to be done when moving from instant T-1 to instant T
%%%
manage_focus(T) :- ground(T),oubli(T),!.
manage_focus(_) :- print('manage_focus: time is not instantiated').

% temporal decay of focus (forget things over time, pay less attention to them, decrease their activation) 
% emotional decay depends on focus decay (all other degrees stay the same over time)
% 0.66 is too quick (already forgotten after two time units)
oubli(T) :- 
		% compute previous instant
		T1 is T-1,
		% propagate after decreasing high enough degrees (over 0.2) by appropriate coefficient
		forall((focus(Agt,Prop,Deg1,T1),Deg1>=0.2), (Deg is Deg1*0.8,set_focus(Agt,Prop,Deg,T))),
		% degrees that are under this threshold (0.2) are outright erased from focus
		forall((focus(Agt,Prop,Deg1,T1),Deg1<0.2), retract(focus(Agt,Prop,Deg1,T1))).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ------- BELIEFS   CLEANING -------     %%
%% to free some space in memory           %%
%% delete beliefs older than 3 time steps %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% retract older beliefs (except planning rules and speech rules)
retract_old_beliefs(_A,T) :- T =< 2,!.
retract_old_beliefs(A,T) :- T > 2,
						  T1 is T - 3,
						  forall((bel(A,P,D,T1,Src),
						  Src \= speech_rule,
						  Src \= planification),
						  retract(bel(A,P,D,T1,Src))).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ------- BELIEFS  PROPAGATION ------- %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% beliefs that were true at T stay true at T+1 (except contrary order) and with the same degree
% past beliefs are NOT retracted (this is memory)
% what is propagated is the bel() predicate stored in KB. 
% believe() and prob() predicates will be deduced from bel() based on degrees

%% IDEA Nov2012 : do not propagate deductions ! (to check)
% so that only things that can still be deduced at the current time
% are found in the KB at each time

maintain_belief(T) :- 
		% compute previous instant
		T1 is T-1,
                maintain_belief_general(T1,T),
                maintain_beliefs_about_beliefs(T1,T).
                

% general case for most beliefs
maintain_belief_general(T1,T) :-
		% for all beliefs at previous time about P
		forall((bel(A,P,D,T1,Src),
		% NEW : do NOT propagate deductions 
		% (should be re-deduced if still possible)
		Src \= deduction,
                % New from Jeremy - do not propagate planning rules and speech rules
                % they are never retracted from instant 0
                Src \= speech_rule, Src \= planification,
                % do not propagate beliefs of others (time needs to be updated)
                P \= believe(_Other,_Prop,_Degree,T1),
		% if there is no belief yet at the new instant about either P or not(P)
		not(bel(A,not(P),_,T,_)),not(bel(A,P,_,T,_))),
		% then re-assert the old belief at the new instant with the same caracs
		assert_belief(A,P,D,T,Src)).

% special case for mental attitudes of other agents: need to change time inside them too
maintain_beliefs_about_beliefs(T1,T) :-
		% for all beliefs at previous time about P
		forall((bel(A,believe(Other,Prop,Degree,T1),D,T1,Src),
		% NEW : do NOT propagate deductions 
		% (should be re-deduced if still possible)
		Src \= deduction,
                % if there is no belief yet at the new instant about either P or not(P)
		not(bel(A,not(believe(Other,Prop,Degree,T1)),_,T,_)),not(bel(A,believe(Other,Prop,Degree,T1),_,T,_))),
		% then re-assert the old belief at the new instant with the same caracs
		assert_belief(A,believe(Other,Prop,Degree,T),D,T,Src)).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ------- GOALS  PROPAGATION ------- %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% NEW JUNE 2013
%% goals are propagated to the next instant too
%% (intentions are not, they will be deduced via planning to take new context into account)
%% FIXME: do we need a source of goals? to be used in priority ordering between emotion goal, etc
% only propagated if not achieved !
% T is the new time instant
maintain_goal(T) :- 
		% compute previous instant
		T1 is T-1, 
		% for all beliefs at previous time about P
		forall((goal(A,P,D,T1,S),
		% if there is no goal yet at the new instant about either P or not(P)
		not(goal(A,not(P),_,T,_)),not(goal(A,P,_,T,_)),
                % and the goal is not believed to be achieved at previous or new current instant
                not(bel(A,P,_,T1,_)),not(bel(A,P,_,T,_))
                ), % end forall condition
                (
		% then re-assert the old goal at the new instant with the same caracs
		assert_goal(A,P,D,T,S),
                if_debug_tag(propag,writeln('propagate goal('+A+P+D+T+S+') to new instant '+T))
                )
                % end forall
                ).
                
% maintain conditional goals as well
maintain_condgoal(T) :- 
		% compute previous instant
		T1 is T-1, 
		% for all beliefs at previous time about P
		forall((condgoal(A,Cond,P,D,T1,S),
		% if there is no goal yet at the new instant about either P or not(P)
		not(goal(A,not(P),_,T,_)),not(goal(A,P,_,T,_)),
                % and the goal is not believed to be achieved at previous or new current instant
                not(bel(A,P,_,T1,_)),not(bel(A,P,_,T,_))
                ), % end forall condition
                (
		% then re-assert the old goal at the new instant with the same caracs
		assert_condgoal(A,Cond,P,D,T,S),
                if_debug_tag(propag,writeln('propagate condgoal('+A+Cond+P+D+T+S+') to new instant '+T))
                )
                % end forall
                ).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ------- Intentions PROPAGATION ------- %%						 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% propagate intentions - from Jeremy
propag_intentions(T) :-
					% previous instant
					T1 is T-1, 
					% find all previous intentions that were not achieved yet
					%% intentions to have a certain mental attitude
					forall((intend(A,bel(A,PP,DD,TT,SS),D,T1,Src),
					not(bel(A,PP,_,_,_))),
					assert_intention(A,bel(A,PP,DD,TT,SS),D,T,Src)),
					%% intentions that something is true (not a mental attitude)
					forall((intend(A,P,D,T1,Src),P \= bel(A,_,_,_,_),
					% only propagate intentions that were not achieved yet
					not(believe(A,P,_,_))),
					%% propagate the intention
					assert_intention(A,P,D,T,Src)).

