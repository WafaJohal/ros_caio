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

% MODIFS Nov2012 - translation and debug and merge
% modif June2014 - intend passe de 4 a 5 params (+source/type d'intention cf Jeremy)

%% list of all strategies
%% problem-focused : active coping, seek material support
%% emotion-focused : denial, positive reinterpretation, mental disengagement, seek emotional support, focus and venting

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%                                  %%%%
%%%%   COPING STRATEGIES MANAGEMENT   %%%%
%%%%                                  %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% modif 200707: added strategies

%%% hyps
%% the coping strategy applies to the most intense negative emotion

% TODO
%% a predicate that checks if a strategy is applicable, without applying it
%% a predicate for choosing the most preferred strategy
%% a predicate for applying the strategy and asserting its effects


% tool for negating the stressor (stimuli that triggered the emotion being coped with)
neg_stresseur(Stresseur,not(Stresseur)) :- atom(Stresseur).
neg_stresseur(not(Antistresseur),Antistresseur) :- atom(Antistresseur).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%   BASIC COND : MOST INTENSE NEGATIVE EMOTION  %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% sorting tool 
% inserts an emotion in its place in a sorted list of emotions (by decreasing intensity)
% base case: insert in an empty list
insert_emo_intensdec(E,[],[E]).
% insert in head position if more intense than the current head element
insert_emo_intensdec(emotion(A,E,P,D,T),[emotion(A1,E1,P1,D1,T1)|L1],
			   [emotion(A,E,P,D,T),emotion(A1,E1,P1,D1,T1)|L1]) :- 
						D>=D1.
% recursively insert in tail if less intense than current head emotion
insert_emo_intensdec(emotion(A,E,P,D,T),[emotion(A1,E1,P1,D1,T1)|L1],
				[emotion(A1,E1,P1,D1,T1)|L2]) :-
						D < D1,
						insert_emo_intensdec(emotion(A,E,P,D,T),L1,L2).

% insert-sort function that uses this insertion predicate
% inserts the first list (element by element) in the second list to form the result list
tri_aux_intensdec([],LTri,LTri).
tri_aux_intensdec([E|LNT],LAux,Ltri) :-      
			insert_emo_intensdec(E,LAux,NLAux),
			tri_aux_intensdec(LNT,NLAux,Ltri).

% main function that initialises parameters for the auxiliary function
tri_intensdec(Liste_Emos,Liste_Triee) :- tri_aux_intensdec(Liste_Emos,[],Liste_Triee).

% function to get the most intense emotion
% sorts the list of emotions and returns its head
% FIXME should be defined in pleiad_emotion file ?
emo_maxintens(Agt,T,E) :- vecteur_emo(Agt,T,V),tri_intensdec(V,[E|_]).

% function to get the most intense negative emotion
% builds the vector of negative emotions, sorts it, returns its head
emoneg_maxintens(Agt,T,E) :- vector_emo_neg(Agt,T,V),
					tri_intensdec(V,[E|__]).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% TYPECOND : CONTROLABILITY OF STIMULUS %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% controlability of stimulus
% the stressor is controllable if the agent believes that there exists some action whose effect is to negate the stressor
control_stimulus(Agt,P,T) :- neg_stresseur(P,NP),bel(Agt,after(_,_,NP),_,T,_).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% tricop tool = sort coping strategies in preference order %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% V1 (in) = applicable strategies
% V2 (in) = preference order (given in personality) 
% V3 (out) = sorted applicable strategies

% base case: no pref order given: resulting vector is empty 
% ie no coping strategy will be selected.. problem??
tricop(_,[],[]).
% recursive case: if the first preferred strategy is applicable (in V)
% then it is placed first in the resulting vector
tricop(V,[P1|VP],[P1|VS]) :- member(P1,V),tricop(V,VP,VS),!.
% if the first preferred strategy is not applicable
% then it is discarded and the next one is considered
tricop(V,[_|VP],VS) :- tricop(V,VP,VS),!.

% idea: parse pref vector, keep applicable strategies in order
% warning: this assumes that pref vector contains ALL strategies
% (ie an applicable strategy that is not in the preference vector will not be ranked,
% which allows an agent to exclude a coping strategy that he does not like)
% but what if he has no choice, ie no other applicable strategy?
% so all agents should have a default pref order to ensure they will cope


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%   GENERAL COPING FUNCTIONS   %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%*******%%% FONCTION 1 : all manual
%%% apply a given strategy (if feasible) on an emotion given by the user
%%% (strategy and emotion specified by user for debug purposes)
apply_one_strategy(Agt,Strat,Emo) :- 
			instant(T),
			% if emotion is instantiated use it
			% otherwise choose the most intense neg emo by default
			(ground(Emo)->true;emoneg_maxintens(Agt,T,Emo)),
			% strategy only needs to be applicable (even if not the best)
			coping_strategy_applycond(Agt,Strat,T,Emo),
			coping_strategy_effect(Agt,Strat,T,Emo),
			saturer_bc(Agt),!.

% if not applicable write a message
apply_one_strategy(Agt,_Strat,_Emo) :- write(Agt),write(' cannot use this strategy now and here').


%%%*******%%% FONCTION 2 : all auto
%%% apply best possible strategy
%%% automatically chosen by the agent based on the context and his personality
%%% only the agent is specified by the user

apply_best_strategy(Agt,Strat,Emo) :-
						instant(T),
						emoneg_maxintens(Agt,T,Emo),
						strategie_preferee(Agt,Strat),
						coping_strategy_effect(Agt,Strat,T,Emo),
						saturer_bc(Agt).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% COPING STRATEGIES : APPLYCOND applicability condition %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Emo MUST be instantiated by the user or by emoneg_maxintens
% before calling coping_strategy_emo

% we distinguish 3 types of stimulus, some strategies only apply to one of these types
% type phi (csq of event), alpha (action), j (agent)

% list of strategies
%% pbfoc: active_coping(Alpha), seek_material_support(Alpha),
%% emofoc: denial(Argument), positive_reinterpretation(Csq), mental_disengagement(Action), shift_responsability(J), 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% PROBLEM-FOCUSED COPING %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% these strategies need a controllable stimulus
% ie there exists an action (and later a plan)
% that allows to cancel the stressor 


%%%%%%%%%%%%%%%%%%%%%
%%% ACTIVE COPING %%%
%%%%%%%%%%%%%%%%%%%%%

coping_strategy_applycond(Agt,active_coping(Alpha),T,E) :-
				% only works on an emotion concerning a proposition Phi
				stresseur_phi(E,Phi),
				neg_stresseur(Phi,NotPhi),
				% controllability (useless since included in addcond here)
				control_stimulus(Agt,Phi,T),
				% addcond: the agent believes he can do sth himself
				bel(Agt,after(Agt,Alpha,NotPhi),_Degbel,T,_).
				

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% SEEK MATERIAL SUPPORT %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

coping_strategy_applycond(Agt,seek_material_support(AutreAgt,Alpha),T,E) :- 
				% only works on an emotion concerning a proposition Phi
				stresseur_phi(E,Phi),
				neg_stresseur(Phi,NotPhi),
				% controllability (again useless since implied by addcond here)
				control_stimulus(Agt,Phi,T),
				% the agent believes that another agent can do sth against phi
				bel(Agt,after(AutreAgt,Alpha,NotPhi),_Degbel,T,_),
				AutreAgt \= Agt.
 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% EMOTION-FOCUSED COPING STRATEGIES %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%
%%% DENIAL %%%
%%%%%%%%%%%%%%

coping_strategy_applycond(Agt,denial(Argument),T,Emo) :- 
				% the stressor
				stresseur_phi(Emo,Phi),

				% condition of type (uncontrollable stimulus)
				not(control_stimulus(Agt,Phi,T)),
				
				% negation of stressor (Phi must be instantiated)
				neg_stresseur(Phi,NotPhi),

				% applicability condition specific to this strategy
				% there must exist an argument in favour of NotPhi
				bel(Agt,Argument->NotPhi,_Deg,T,_).

% idea : an agent using denial should avoid further information about the topic to avoid
% getting further evidence of what he is denying
				

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% POSITIVE REINTERPRETATION %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

coping_strategy_applycond(Agt,positive_reinterpretation(Csq),T,Emo) :-
				% thee stressor
				stresseur_phi(Emo,Phi),
				% condition of control
				not(control_stimulus(Agt,Phi,T)),

				% particular condition : there exists a consequence of the stressor
				bel(Agt,Phi->Csq,_Degbel,T,_),
				% that is not undesirable (so slightly positive)
				not(undesire(Agt,Csq,_)).
				

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% MENTAL DISENGAGEMENT %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

coping_strategy_applycond(Agt,mental_disengagement(Action),T,Emo) :-
				% the stressor
				stresseur_phi(Emo,Phi),
				not(control_stimulus(Agt,Phi,T)),
	
				% particular condition : an action exists
				bel(Agt,after(Agt,Action,Rez),_Degbel,T,_),
				% with a desirable result
				desire(Agt,Rez,_Degdes).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% SEEK EMOTIONAL SUPPORT %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

coping_strategy_applycond(Agt,seek_emotional_support(AutreAgt),T,Emo) :-
				% the stressor
				stresseur_phi(Emo,Phi),
				not(control_stimulus(Agt,Phi,T)),
				% particular condition : it is a friend who will believe what we feel
				ami(Agt,AutreAgt,_).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% FOCUS AND VENTING %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% the agent only wants to express the emotion triggered by phi to a witness (who also believes phi)
% (but may not feel any (or the same) emotion about it)

coping_strategy_applycond(Agt,focus_and_venting(AutreAgt),T,Emo) :-
				% the stressor
				stresseur_phi(Emo,Phi),
				not(control_stimulus(Agt,Phi,T)),
				% particular condition: it is a witness of the stressor
				bel(Agt,believe(AutreAgt,Phi,_,T),_Deg,T,_).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% COPING STRATEGIES : BEST PARAM %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% search the best parameter for each type of coping strategy
% (if it could be applied on several different arguments)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% PROBLEM-FOCUSED COPING %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%
%%% ACTIVE COPING %%%
%%%%%%%%%%%%%%%%%%%%%

% find the best action Alpha if several are possible
coping_strategy_best(Agt,active_coping(BestAlpha),T,E) :-
				stresseur_phi(E,Phi),
				neg_stresseur(Phi,NotPhi),
				findall((Alpha,Deg),
					   (coping_strategy_applycond(Agt,active_coping(Alpha),T,E),
					    bel(Agt,after(Agt,Alpha,NotPhi),Deg,T,_)),
					    VAlphaDeg),
				% rank actions by degree of belief in their consequence
				tripardeg_decroissant(VAlphaDeg,[(BestAlpha,_Deg)|_]).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% SEEK MATERIAL SUPPORT %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

coping_strategy_best(Agt,seek_material_support(BestAgt,BestAlpha),T,E) :-
	stresseur_phi(E,Phi),
	neg_stresseur(Phi,NotPhi),
	findall(((AutreAgt,Alpha),Deg),
	% all applicable strategies
	(coping_strategy_applycond(Agt,seek_material_support(AutreAgt,Alpha),T,E),
	% degree for comparison	    
	bel(Agt,after(AutreAgt,Alpha,NotPhi),Deg,T,_)),
	% vector of pairs (Param,Deg)
	VAlphaDeg),
	% rank the pairs (AutreAgt,Action) by associated degree 
	% the first one is the best: choose that one
	tripardeg_decroissant(VAlphaDeg,[((BestAgt,BestAlpha),_Deg)|_]).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% EMOTION-FOCUSED COPING STRATEGIES %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%
%%% DENIAL %%%
%%%%%%%%%%%%%%

coping_strategy_best(Agt,denial(BestArgument),T,Emo) :-
				stresseur_phi(Emo,Phi),
				neg_stresseur(Phi,NotPhi),
				findall((Argument,Deg),
						% all applicable strategies
					   (coping_strategy_applycond(Agt,denial(Argument),T,Emo),
						% the degree on which they will be compared 
					    bel(Agt,Argument->NotPhi,Deg,T,_)),
						% the vector of pairs (Param,Deg)
					    VAlphaDeg),
				% rank the pairs by associated degree (first one is best)
				tripardeg_decroissant(VAlphaDeg,[(BestArgument,_Deg)|_]).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% POSITIVE REINTERPRETATION %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

coping_strategy_best(Agt,positive_reinterpretation(BestCsq),T,Emo) :-
				stresseur_phi(Emo,Phi),
				findall((Csq,Deg),
						% all applicable strategies
					   (coping_strategy_applycond(Agt,positive_reinterpretation(Csq),T,Emo),
						% degree to compare them
					    bel(Agt,Phi->Csq,Deg,T,_)),
						% vector of pairs (Param,Deg)
					    VAlphaDeg),
				% sort all pairs
				tripardeg_decroissant(VAlphaDeg,[(BestCsq,_Deg)|_]).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% MENTAL DISENGAGEMENT %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

coping_strategy_best(Agt,mental_disengagement(BestAction),T,Emo) :-
				findall((Action,Deg),
						% all applicable strategies
					   (coping_strategy_applycond(Agt,mental_disengagement(Action),T,Emo),
						% degree of comparison
						bel(Agt,after(Agt,Action,Rez),Degbel,T,_),
						desire(Agt,Rez,Degdes),
						Deg is Degbel*Degdes),
						% vector of pairs (Param,Deg)
					    VAlphaDeg),
				% sort the pairs
				tripardeg_decroissant(VAlphaDeg,[(BestAction,_Deg)|_]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% SEEK EMOTIONAL SUPPORT %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

coping_strategy_best(Agt,seek_emotional_support(BestAmi),T,Emo) :-
				findall((Ami,Deg),
						% all applicable strategies
					   (coping_strategy_applycond(Agt,seek_emotional_support(Ami),T,Emo),
						% degree for comparison
						ami(Agt,Ami,Deg)),
						% resulting vector of pairs (Param,Deg)
					    VAlphaDeg),
				% sort pairs
				tripardeg_decroissant(VAlphaDeg,[(BestAmi,_Deg)|_]).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% FOCUS AND VENTING    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ranks based on the agent's certainty that the other agent witnessed phi
coping_strategy_best(Agt,focus_and_venting(BestAutreAgt),T,Emo) :-
				stresseur_phi(Emo,Phi),
				findall((AutreAgt,Deg),
						% all applicable strategies
					   (coping_strategy_applycond(Agt,focus_and_venting(AutreAgt),T,Emo),
						% degree for comparison
					    bel(Agt,believe(AutreAgt,Phi,Deg1,T),Deg2,T,_),
					    Deg is Deg1*Deg2),
						% vector pairs (Param,Deg)
					    VAlphaDeg),
				% sort the pairs
				tripardeg_decroissant(VAlphaDeg,[(BestAutreAgt,_Deg)|_]).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% COPING STRATEGIES : EFFECTS    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% PROBLEM-FOCUSED COPING %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%
%%% ACTIVE COPING %%%
%%%%%%%%%%%%%%%%%%%%%

coping_strategy_effect(Agt,active_coping(Alpha),T,emotion(_,_,_,D,_)) :-
				%% effect of strategy:
				% the agent adopts the intention to do what he can do
				% prio = intensity of emotion
				assert_intention(Agt,done(Agt,Alpha,_),D,T,coping)).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% SEEK MATERIAL SUPPORT %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

coping_strategy_effect(Agt,seek_material_support(AutreAgt,Alpha),T,emotion(_,_,_,Deg,_)) :-
				%% effect of strategy: the agent adopts the intention
				%% to request help from the other agent that is believed to be able to help
				assert_intention(Agt,done(Agt,request(Agt,AutreAgt,done(AutreAgt,Alpha,_),Deg),_),Deg,T,coping).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% EMOTION-FOCUSED COPING STRATEGIES %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%
%%% DENIAL %%%
%%%%%%%%%%%%%%

coping_strategy_effect(Agt,denial(Argument),T,emotion(_,_,_,Deg,_)) :-
				% effect1: Agt retracts the belief that Phi is false
				% -> the strategy is then not applicable anymore
				retract(bel(Agt,Phi,_,_,_)),

				% effect2: Agt persuades himself that Argument is true
				kassert(bel(Agt,Argument,Deg,T,conviction)),				% warning: this belief is of type "deduction" (lowest priority) TODO deduction or conviction?

				% the agent should focus less on Phi to avoid new contradictory info
				% (when perception mechanism depends on focusÉ TODO)
				infocus(Agt,Phi,Old_deg,T),
				New_deg is Old_deg/4,
				set_focus(Agt,Phi,New_deg,T).
				% also makes that joy is not felt (too low intensity) otherwise not realistic




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% POSITIVE REINTERPRETATION %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

coping_strategy_effect(Agt,positive_reinterpretation(Csq),T,emotion(_,_,_,Deg,_)) :-
				% effect1: make the consequence desirable with the same degree as the emotion being coped with
				kassert(desire(Agt,Csq,Deg)),

				% effect2: focus on Csq instead of on Phi
				infocus(Agt,Csq,Degfoc,T),
				augmenter_degre_beaucoup(Degfoc,NDegfoc),
				set_focus(Agt,Csq,NDegfoc,T),

				infocus(Agt,Phi,Degfoc2,T),
				diminuer_degre_beaucoup(Degfoc2,NDegfoc2),
				set_focus(Agt,Phi,NDegfoc2,T).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% MENTAL DISENGAGEMENT %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

coping_strategy_effect(Agt,mental_disengagement(Action),T,emotion(_,_,_,Deg,_)) :-
				% the agent adopts the intention to perform the disengaging action
				% prio = intensity of triggering emotion
				assert_intention(Agt,done(Agt,Action,_),Deg,T,coping).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% SEEK EMOTIONAL SUPPORT %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% the agent wants a friend to be sorry for him, to feel compassion, he wants to provoke empathy
% either this other agent knows about the agent's desires, or he knows about his beliefs, or nothing
% so the agent has to communicate the missing information to trigger compassion in him

coping_strategy_effect(Agt,seek_emotional_support(AutreAgt),T,emotion(Agt,E,P,Deg,T)) :-
				% the stressor
				stresseur_phi(emotion(Agt,E,P,Deg,T),Phi),
				neg_stresseur(Phi,NegPhi),
				% case n°1: the friend does not know the situation
				(not(bel(Agt,believe(AutreAgt,Phi,_,T),_,T,_))
				-> assert_intention(Agt,done(Agt,inform(Agt,AutreAgt,Phi,Deg),_),Deg,T,coping);true),
				% case n°2: the friend does not know about the contradicted desire
				((desire(Agt,NegPhi,DegDes),not(bel(Agt,believe(AutreAgt,desire(Agt,NegPhi,_),_,T),_,T,_)))
				-> assert_intention(Agt,done(Agt,inform(Agt,AutreAgt,desire(Agt,NegPhi,DegDes),Deg),_),Deg,T,coping);true).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%   FOCUS AND VENTING    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% the effect is to adopt the intention to directly express the felt emotion

coping_strategy_effect(Agt,focus_and_venting(AutreAgt),T,emotion(Agt,E,P,Deg,T)) :-
				% assert the intention to inform this other agent of his emotion
				assert_intention(Agt,done(Agt,inform(Agt,AutreAgt,emotion(Agt,E,P,Deg,T),Deg),_),Deg,T,coping).






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%  CHOICE OF PREFERRED STRATEGY  %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% choice among applicable strategies depending on the agent's personality

% preferred strategy of agent A (regarding its most intense negative emotion at current time)
% build vector of strategies and sort them (by decreasing preference), return head (most preferred)
strategie_preferee(A,S) :- vstrat(A,V),vstrat_sort(A,V,[S|_]).

% vector of all applicable strategies against agent A's most intense negative emotion at current instant
% (keeping only the best strategy of each type)
vstrat(Agt,VStrat) :- 
		instant(T),
		emoneg_maxintens(Agt,T,Emo),
		findall(Strat,coping_strategy_best(Agt,Strat,T,Emo),VStrat).

% sort / rank strategies in order of decreasing preference based on the agent's personality
vstrat_sort(Agt,VStrat,Vsort) :- 
		personal_prefcop(Agt,Vpref),
		tricop(VStrat,Vpref,Vsort).