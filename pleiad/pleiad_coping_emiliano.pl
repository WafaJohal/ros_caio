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
% from Emiliano Lorini and Mehdi Dastani's paper at AAMAS 2012
% A logic of emotions: from appraisal to coping

% HYP: only coping with the most intense negative emotion

% new June 2014 - intend with additional param source/type


% new Nov2012 for coping (emiliano)
rule(200,T,Agt,
	% preconds
		([
			% agent has a desire that Prop be true (todo: a goal?)
			desire(Agt,Prop,Dgoal),
			% and is committed on that desire (so it was highest priority and not abandoned yet)
			committed(Agt,Prop,Dcom,T),
			% agent has an action leading to goal being true
			bel(Agt,after(Agt,Alpha,Prop),Dbel,T,_),
			% and this action hasnt been abandoned already
			not(abandoned(Agt,Alpha,T)),
			% compute degree	
			Dprio is Dgoal*Dbel*Dcom]),
	% csq: adopt goal to perform action making desire true (source = task)
			goal(Agt,done(Agt,Alpha,_),Dprio,T,task)
).


% NEW nov2012
%rule(200,T,Agt,
%	% preconds
%	(
%		% 
%		
%	)
%	% conclusion	
%).




%%%%%%%%%%%%%%%%%%%%%%%%
% MOST INTENSE EMOTION %
%%%%%%%%%%%%%%%%%%%%%%%%

% find most intense NEGATIVE emotion
% (to check that effect of coping strategy is to change that)
most_intense_emotion(A,eemotion(A,E,P,Max,T),T) :-
	% find all degrees of all emotions (only negative)
	findall(Deg,(eemotion(A,E,P,Deg,T),negative_emotion(E)),ListDeg),
	% sort the list of degrees (increasing order) 
	% and get last element = max degree
	sort(ListDeg,SL),length(SL,N),nth1(N,SL,Max),
	% retrieve emotion(-s-) having exactly that degree
	% there could be several emotions with same max degree
	eemotion(A,E,P,Max,T).	


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ABBREVIATIONS for EMOTIONS %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% negative emotions 

negative_emotion(fear).
negative_emotion(distress).


% compatibility predicate

eemotion(A,fear,[Alpha,Phi],Deg,T) :- fear(A,Alpha,Phi,Deg,T).
eemotion(A,hope,[Alpha,Phi],Deg,T) :- hope(A,Alpha,Phi,Deg,T).
eemotion(A,joy,[Alpha,Phi],Deg,T) :- joy(A,Alpha,Phi,Deg,T).
eemotion(A,distress,[Alpha,Phi],Deg,T) :- distress(A,Alpha,Phi,Deg,T).

% abbreviation with discretisation of scale, hiding time, and renaming predicates to match the logic
emo(A,E,P,D) :- instant(T),eemotion(A,E,P,DD,T),round(10*DD,D).

avdg(A,P,D) :- undesire(A,P,DD),round(10*DD,D).
achg(A,P,D) :- desire(A,P,DD),round(10*DD,D).

int(A,P,D) :- instant(T), intend(A,P,DD,T,_S), round(10*DD,D).

b(A,P,D) :- instant(T), bel(A,P,DD,T,_Src), round(10*DD,D).

list_coping_strategy(A,Strat) :- instant(T),list_coping_strategies(A,L,T),member(Strat,L).

most_intense_emotion(A,emo(A,E,P,D)) :- instant(T),most_intense_emotion(A,eemotion(A,E,P,DD,T),T),round(10*DD,D).

% abbreviation
most_intense_desire(Agt,Prop,MyDeg) :-
                % findall desires not abandoned yet 
		findall(DegDes,
			(desire(Agt,P,DegDes),not(abandoned(Agt,P,_T))), 
			LDeg),
		% sort by (increasing) degree
		sort(LDeg,ST),
		% get highest degree
		length(ST,N),N>0,nth1(N,ST,MyDeg),
		% get desire of that degree (most intense one)
		desire(Agt,Prop,MyDeg).


%%%%%%%%%%%%%%%%%%%%%%%%%%
%% EMOTIONS DEFINITIONS %%
%%%%%%%%%%%%%%%%%%%%%%%%%%

% fear 
% probable undesirable consequence (avoidance goal) of intended action
fear(Agt,Alpha,Phi,Deg,T) :-
	prob(Agt,after(Agt,Alpha,Phi),D1,T),
	undesire(Agt,Phi,D2),
	intend(Agt,done(Agt,Alpha,_),D3,T,_S),
	Deg is D1*D2*D3.

% distress
% certain undesirable consequence of intended action
distress(Agt,Alpha,Phi,Deg,T) :-
	believe(Agt,after(Agt,Alpha,Phi),D1,T),
	undesire(Agt,Phi,D2),
	intend(Agt,done(Agt,Alpha,_),D3,T,_S),
	Deg is D1*D2*D3.

% hope
% desirable consequence (achievement goal) of intended action
hope(Agt,Alpha,Phi,Deg,T) :-
	prob(Agt,after(Agt,Alpha,Phi),D1,T),
	desire(Agt,Phi,D2),
	intend(Agt,done(Agt,Alpha,_),D3,T,_S),
	Deg is D1*D2*D3.

% joy
% certain desirable consequence of intended action
joy(Agt,Alpha,Phi,Deg,T) :-
	believe(Agt,after(Agt,Alpha,Phi),D1,T),
	desire(Agt,Phi,D2),
	intend(Agt,done(Agt,Alpha,_),D3,T,_S),
	Deg is D1*D2*D3.


%%%%%%%%%%%%%%%%%%%%%%%
%% COPING CONDITIONS %%
%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%
% controllability %
%%%%%%%%%%%%%%%%%%%

% tool for negating the stressor (stimuli that triggered the emotion being coped with)
neg_stresseur(Stresseur,not(Stresseur)) :- atom(Stresseur),!.
neg_stresseur(not(Antistresseur),Antistresseur) :- atom(Antistresseur),!.

% control by doing an action (any agent, any action)
control(Agt,Phi,T) :- neg_stresseur(Phi,NotPhi),
			bel(Agt,after(_AnAgt,_AnAlpha,NotPhi),_Degbel,T,_).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% relevance to a particular emotion %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% check that a strategy acts against a particular emotion
% (concretely: the most intense one)

relevance(strategy(_Type,Alpha,Phi),eemotion(_Agt,_Emo,[Alpha,Phi],_Deg,_T)) :- !.
	% a strategy is relevant to any emotion that concerns the same Alpha,Phi 
%	most_intense_emotion(Agt,eemotion(Agt,_Emo,[Alpha,Phi],Deg,T),T),!.


%%%%%%%%%%%%%%%%%%%%%%%
%% REVISION OPERATORS %
%%%%%%%%%%%%%%%%%%%%%%%

% multiply old degree of belief by a coefficient (to decrease degree)
revise_belief(Agt,Prop,Time,Coeff) :-
	% retract old belief
	retract(bel(Agt,Prop,DegOld,Time,Src)),
	% compute new degree
	DegNew is min(1,DegOld*Coeff),
	% assert same belief with new degree
	assert(bel(Agt,Prop,DegNew,Time,Src)).

% should not fail
revise_belief(_Agt,_Prop,_Time,_Coeff) :- !.

% revise desire by increasing degree
% time instant is useless here since desires are supposed permanent
revise_undesire(Agt,Prop,_Time,Coeff) :-
	% retract old desire
	retract(undesire(Agt,Prop,DegOld)),
	% compute new degree from old degree and coeff (warning: stay between 0 and 1)
	DegNew is min(1,DegOld*Coeff),
	% assert same desire with new degree
	assert(undesire(Agt,Prop,DegNew)).

% revision operators should not fail
revise_undesire(_Agt,_Prop,_Time,_Coeff) :- !.

% revise intention : abandon it and remember it was abandon (for replanning purposes)
revise_intention(Agt,Alpha,Time,_Coeff) :-
	% retract intention
	retract(intend(Agt,done(Agt,Alpha,_),_DegIntend,Time,_Src)),
	% assert it was abandoned at that time - src is not saved
	assert(abandoned(Agt,Alpha,Time)).




%%%%%%%%%%%%%%%%%%%%%%%%%%
% COPING CHOICE FUNCTION %
%%%%%%%%%%%%%%%%%%%%%%%%%%

% format for a strategy : strategy(Type,Alpha,Phi)

% findall strategies against most intense emotion
list_coping_strategies(Agt,LStrat,Time) :-
	% most intense emotion of the agent at that time
	most_intense_emotion(Agt,Emo,Time),
	% findall applicable strategies relevant to that emotion at Time
	findall(Strat,(relevance(Strat,Emo),coping_condition(Agt,Strat,Time)),LStrat),!.

% pick one random strategy (no backtrack) against most intense emotion
choose_random_strategy(Agt,MyStrat,Time) :-
	list_coping_strategies(Agt,LS,Time),
	% count applicable relevant strategies
	length(LS,N),
	% fail already of no available strategy
	N>0, 
	% draw a random index (between 1 and N, warning random is between 0 and N-1)
	% warning: cannot backtrack on random...
	I is random(N)+1,
	% pick strategy at that index
	nth1(I,LS,MyStrat),!.

% pick a strategy by type
choose_strategy_by_type(Agt,strategy(StratType,Action,Prop),Time) :-
	list_coping_strategies(Agt,LS,Time),
	member(strategy(StratType,Action,Prop),LS).

% if strategy type is known: instantiate parameters and apply it
cope(Agt,strategy(StratType,Action,Prop)) :-
	% the type of strategy wanted is known
	ground(StratType),
	% choose a strategy of that type in relevant applicable ones
	% to instantiate the other parameters if needed
	choose_strategy_by_type(Agt,strategy(StratType,Action,Prop),Time),
	% apply that strategy
	coping_effect(Agt,strategy(StratType,Action,Prop),Time),!.

% if strategy type is not instantiated: pick a random strategy
cope(Agt,MyStrat) :-
	% current instant
	instant(Time),
	% choose a random strategy (for now, todo: from personality)
	choose_random_strategy(Agt,MyStrat,Time),
	% apply it (condition was already checked so directly apply effects)
	coping_effect(Agt,MyStrat,Time),!.

	


%%%%%%%%%%%%%%%%%%%%%%%%%
% triggering conditions %
% of coping strategies  %
%%%%%%%%%%%%%%%%%%%%%%%%%

%coping_condition(Agt,Strat,Time,Emo)

%coping_condition(Agt,resignation(Alpha),Time,Emo) :-
%	% agent has intention (now) to do action alpha
%	inten-d(Agt,done(Agt,Alpha,_Taction),_DegIntend,Time),
%	% and believes (now) alpha will lead to consequence phi
%	bel(Agt,after(Agt,Alpha,Phi),_DegBel,Time,_SrcBel),
%	% and Agt has a negative emotion about phi
%	stresseur_phi(Emo,Phi),
%	negative_emotion(Emo),	
%	% and Phi is controllable by doing a different action
%	control(Agt,Phi,Time).

% resignation (intention focused : abandon intention to do action having stressful consequence)
coping_condition(Agt,strategy(resignation,Alpha,Phi),T) :-
	(fear(Agt,Alpha,Phi,Deg,T);
	 distress(Agt,Alpha,Phi,Deg,T)),
	control(Agt,Phi,T).

% wishful thinking (belief focused : decrease degree of stressful belief)
coping_condition(Agt,strategy(wishful_thinking,Alpha,Phi),T) :-
	(fear(Agt,Alpha,Phi,Deg,T);
	 distress(Agt,Alpha,Phi,Deg,T)),
	not(control(Agt,Phi,T)).

% positive reinterpretation (desire focused : increase degree of desirability of stressful consequence)
coping_condition(Agt,strategy(positive_reinterpretation,Alpha,Phi),T) :-
	(fear(Agt,Alpha,Phi,Deg,T);
	 distress(Agt,Alpha,Phi,Deg,T)),
	not(control(Agt,Phi,T)).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% effects of coping strategies %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% apply_coping_effect(Agt,Strat,Time)

apply_coping(Agt,Strat,Time) :- 
	% first check condition (if not applicable, should fail)
	coping_condition(Agt,Strat,Time),
	% then apply effects
	coping_effect(Agt,Strat,Time),!.


% resignation : abandon intention
coping_effect(Agt,strategy(resignation,Alpha,_Phi),Time) :-
	revise_intention(Agt,Alpha,Time,0).


% wishful thinking : decrease strength of belief
% bel(Agt,after(Agt,Alpha,Phi)
coping_effect(Agt,strategy(wishful_thinking,Alpha,Phi),Time) :-
	% arbitrary coefficient : divide strength by 2
	revise_belief(Agt,after(Agt,Alpha,Phi),Time,0.5).	


% positive reinterpretation : increase strength of desire / decrease undesire
% undesire(Agt,Phi,D2),
coping_effect(Agt,strategy(positive_reinterpretation,_Alpha,Phi),Time) :-
	revise_undesire(Agt,Phi,Time,0.5).






