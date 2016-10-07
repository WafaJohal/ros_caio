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


% pleiad_demo.pl contains all reasoning rules (inference rules)
% pleiad_reasoning contains the reasoning engine that uses these rules
% 		   and can be affected by panic (activation threshold for attitudes)
%		   and by interest (topic similarity for consideration of mental attitudes)

%% TODO Nov 2012
%% give rules a number so they can easily be activated or de-activated,
%% maybe even use different sets of rules for different agents

%% NEW June2014 - intention with src

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% RULE PREDICATE IS MULTIFILE                       %%
%% in pleiad_demo but also in the additional modules %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

:- multifile(rule/5).


%%%%%%%%%%%%
%% FORMAT %%
%%%%%%%%%%%%

% rule(Index,Time,Agent,Premisse,Conclusion)
% Remark : do not use believe() in premises of rules but bel() (covers both believe and prob)
% Remark : do not use believe() in conclusion, the only predicate that should be asserted in KB is bel() (then believe and prob are deduced from it)

%% !!! warning
%% the deduction predicate is written in a way that allows only one conclusion
%% (write one rule per conclusion for the same premises)

% remark: these rules can create contradictory desires or undesires
% and thus contradictory emotions, which is no problem

%% TODO Nov2012
%% rules with negative effect : effect is not(P)
%% reasoning engine should retract P


%%%%%%%%%%%%%
%%% IDEAS %%%
%%%%%%%%%%%%%

%% NEW Nov2012
% time instant parameter in the rule to optimise saturation by:
%   - avoiding deducing again things from old mental attitudes that are kept in memory
% 	ie only applying rules to the attitudes at the current instant T
%   - not having to specify instant(T) in each rule (now automatic)

% NEW Nov2012
% index of rules to list active rules by index
% agent parameter to saturate only a specific agent KB (eg on loading it)



%%%%%%%%%%%%%%%%%%%%%
%%% list of rules %%%
%%%%%%%%%%%%%%%%%%%%%

% list indexes of rules that should be used
active_rules([1,2,3,4,5,6,7,8,9,10,15,16,17,18,20,22,23]).

%active_rules([1]) :- !.

% list indexes of rules that should not be used
inactive_rules([19,21,11,12,13,14]) :- !.



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% FRIENDSHIP :               %% 
%% propagation of (un)desires %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% friendship rule for desires
rule(1,T,Agt1,
	% premise: Agt1 believes at current instant T that friend Agt2 desires Prop
	([ami(Agt1,Agt2,DegAm),
	bel(Agt1,desire(Agt2,Prop,DegDes),DegBel,T1,_),
	T1=<T,Deg is DegAm*DegDes*DegBel]),  % ,!
	% conclusion: Agt1 desires that Agt2 believes Prop
	desire(Agt1,believe(Agt2,Prop,_,_),Deg)).

% friendship rule for undesires
rule(2,T,Agt1,
	% premise: Agt1 believes that friend Agt2 undesires Prop
	([ami(Agt1,Agt2,DegAm),
	bel(Agt1,undesire(Agt2,Prop,DegDes),DegBel,T1,_),
	T1=<T,Deg is DegAm*DegDes*DegBel]), % ,!
	% conclusion: Agt1 undesires that Agt2 believes Prop
	undesire(Agt1,believe(Agt2,Prop,_,_),Deg)).
	
% unfriendship rule for desire
rule(3,T,Agt1,
	% premise: Agt1 believes that enemy Agt2 desires Prop
	([ennemi(Agt1,Agt2,DegAm),
	bel(Agt1,desire(Agt2,Prop,DegDes),DegBel,T1,_),
	T1=<T,Deg is DegAm*DegDes*DegBel]), % ,!
	% conclusion : Agt1 undesires that Agt2 believes Prop
	undesire(Agt1,believe(Agt2,Prop,_,_),Deg)).
	
% unfriendship rule for undesires
rule(4,T,Agt1,
	% premise : Agt1 believes that enemy Agt2 undesires Prop
	([ennemi(Agt1,Agt2,DegAm),
	bel(Agt1,undesire(Agt2,Prop,DegDes),DegBel,T1,_),
	T1=<T,Deg is DegAm*DegDes*DegBel]), % ,!
	% conclusion : Agt1 desires that Agt2 believes Prop
	desire(Agt1,believe(Agt2,Prop,_,_),Deg)).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   AXIOMATICS  of  LOGIC             %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% axiomatics of the logic             %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MODUS PONENS AND VARIATIONS %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% modus ponens
rule(51,T,Agt,
	([bel(Agt,PropA,DA,T,_),
	bel(Agt,PropA->PropB,DI,T,_),
	D is min(DA,DI)
	]),
	% conclusion
	bel(Agt,PropB,D,T,deduction)).

% modus ponens converse / contraposee
% rule(5,T,Agt,
% 	(bel(Agt,not(PropB),DA,T,_),
%	bel(Agt,PropA->PropB,DI,T,_),
%	D is min(DA,DI)
%	),
%	% conclusion
%	bel(Agt,not(PropA),D,T,deduction)).

% simpler version of the converse (that allows to deduce previous version in 2 steps)
rule(52,T,Agt,
 	([
	bel(Agt,PropA->PropB,DI,T,_),
        % to avoid infinite application of this rule, adding imbricated not() around the conclusion
        atom(PropB)]
	),
	% conclusion
	bel(Agt,not(PropB)->not(PropA),DI,T,deduction)
).


% awareness of modus ponens for others
rule(53,T,Agt,
	([bel(Agt,believe(Other,PropA,DOA,T),DAA,T,_),
	bel(Agt,believe(Other,PropA->PropB,DOI,T),DAI,T,_),
	DO is DOA* DOI,
       DA is DAA * DAI
	]),
    % conclusion
    bel(Agt,believe(Other,PropB,DO,T),DA,T,deduction)
).

% awareness of the converse
rule(54,T,Agt,
	([bel(Agt,believe(Other,not(PropB),DOA,T),DAA,T,_),
	bel(Agt,believe(Other,PropA->PropB,DOI,T),DAI,T,_),
	DO is DOA* DOI,
       DA is DAA * DAI
	]),
    % conclusion
    bel(Agt,believe(Other,not(PropA),DO,T),DA,T,deduction)
).


%%%%%%%%%%%%%%%%%%%%%%%%
%% CONDITIONAL GOALS  %%
%%%%%%%%%%%%%%%%%%%%%%%%

rule(55,T,Agt,
    ([
        % conditional goal
        condgoal(Agt,Condition,Goal,Deg,T,Src),
        % condition is believed to be true
        bel(Agt,Condition,_Deg,T,_Src)
    ]),
    % conclusion: assert the goal
    goal(Agt,Goal,Deg,T,Src)
).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DESIRES and UNDESIRES  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% link between desire end undesire
%% time does not matter here (permanent desires)
rule(6,_T,A,([desire(A,P,D),atom(P)]),undesire(A,not(P),D)).
rule(7,_T,A,([desire(A,not(P),D),atom(P)]),undesire(A,P,D)).
rule(8,_T,A,([undesire(A,P,D),atom(P)]),desire(A,not(P),D)).
rule(9,_T,A,([undesire(A,not(P),D),atom(P)]),desire(A,P,D)).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%          PLANNING           %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% axiome PLAN pour la planif  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% the agent adopts the goal to perform an action that results in (or could result in)
% an effect that he aims/has the goal to achieve

rule(10,T,Agt,
	% preconds
			([
			bel(Agt,after(Agt,Alpha,Effet),Dbel,T,_),
			% intention to obtain this effect
			goal(Agt,Effet,Degprio,T,Src),
			Dprio is Degprio*Dbel
			]),
	% csqs
			goal(Agt,done(Agt,Alpha,_),Dprio,T,Src)
).

% rule(10,T,Agt,
%	% preconds
%			([bel(Agt,after(Agt,Alpha,Effet),Dbel,T,_),
%			% intention to obtain this effect
%			intend(Agt,Effet,Degprio,T,_Src),
%			Dprio is Degprio*Dbel]),
%	% csqs - same src or src=planif?
%			intend(Agt,done(Agt,Alpha,_),Dprio,T,planif)
% ).


% todo nov2012 : pleiad_planif, cf algo planif fait avec Jeremy (October 2012)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% AXIOM PLAN - SPECIAL CASES   %%
%% from goals to goals          %%
%% handling knowval and knowif  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% rules 10b and 10c : goals to gain a special mental attitude (knowif or knowval, of self or other agent)
%            since these are abbreviations, they cannot be directly the effect of any action
% --> find the appropriate speech act (FIXME: what about physical perception actions, like look/observe?)
% then rule 10c/103 will find the associated loc act? 

%%%
% USELESS !!!
% FIXME: no speech act directly has these beliefs as effect, need trust rules to be deduced
%%%

% goal to knowval : find speech act with effect to believe(value(...))
% rule(111,T,Agt,
%    % preconds
%    ([
%        % goal to knowval
%        goal(Agt,knowval(Agt,Prop,_Time),Deg,T,_Src),
%        % action after which the agent believes some value of prop
%        bel(Agt,after(Agt,Alpha,believe(Agt,value(Prop,_Val),DegVal,_TimeVal)),Dbel,T,_),
%        % compute degree
%        Dprio is Deg*DegVal*Dbel
%    ]),
%    % conclu - new goal to perform that action (intention will be asserted from most prio goal only)
%    goal(Agt,done(Agt,Alpha,_),Dprio,T)
% ).

% goal to knowif : find speech act with effect to believe(P) or to believe(not(P))
% rule(112,T,Agt,
%    % preconds
%    ([
%        % goal to knowval
%        goal(Agt,knowvif(Agt,Prop,_Time),Deg,T,_Src),
%        % action after which the agent believes some value of prop
%        (bel(Agt,after(Agt,Alpha,believe(Agt,Prop,DegVal,_TimeVal1)),Dbel,T,_) ;
%         bel(Agt,after(Agt,Alpha,believe(Agt,not(Prop),DegVal,_TimeVal2)),Dbel,T,_) ),
%        % compute degree
%        Dprio is Deg*DegVal*Dbel
%    ]),
%    % conclu
%    goal(Agt,done(Agt,Alpha,_),Dprio,T)
% ).


% from goals to intentions -> moved to pleiad_delib - june2014



%%%%%%%%%%%%%%%%%%%%%
%% desire planning %%
%%%%%%%%%%%%%%%%%%%%%

%%%
% NEW from Jeremy - to be tested
% should only be true for goals or even commitments
% add desires and undesires (if not existing yet)
% on preconditions of desired or undesired states
%%%

% rules 11-12-13-14 commented July 12, 2013 because create an infinite loop
% when saturating KB of the agent if relaxing the D>0.8 condition for beliefs (from asserted bel predicate)
% in pleiad_kbmgr.pl

%% INACTIVE
% a precondition of a desired state is desired
% rule(11,_T,Agt,
%	% preconditions
%	([believe(Agt,Precond->Etat,Deg1,_),
%	desire(Agt,Etat,Deg2),
%	Deg is Deg1*Deg2,
%	not(desire(Agt,Precond,_))
%       ]),
%	% conclusion
%	desire(Agt,Precond,Deg)).

% its negation is undesired
% rule(12,_T,Agt,
%	% preconditions
%	([
%            believe(Agt,Precond->Etat,Deg1,_),
%            desire(Agt,Etat,Deg2),
%            Deg is Deg1*Deg2,
%            not(undesire(Agt,not(Precond),_))
%        ]),
%	% conclusion
%	undesire(Agt,not(Precond),Deg)).

% a precondition of an undesired state is undesired
% rule(13,_T,Agt,
%	% premises
%	([
%            believe(Agt,Precond->Etat,Deg1,_),
%            undesire(Agt,Etat,Deg2),
%            Deg is Deg1*Deg2,
%            not(undesire(Agt,Precond,_))
%        ]),
%	% conclusion
%	undesire(Agt,Precond,Deg)).

% its negation is desired
% rule(14,_T,Agt,
%	% premises
%	([believe(Agt,Precond->Etat,Deg1,_),
%	undesire(Agt,Etat,Deg2),
%	Deg is Deg1*Deg2,
%	not(desire(Agt,not(Precond),_))
%       ]),
%	% conclusion
%	desire(Agt,not(Precond),Deg)).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%         ACTION TENDENCIES PLANNING          %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% rules to handle action tendencies                   %%%
%%% deduction of an intention directly from an emotion  %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ND = 3D/4 so that the action tendency has less priority than the coping strategy
% TODO later: this will depend on the agent's personality
% Rk October 2012 : why not have both? shouldn't the action tendency be automatic and high priority?
% Rk Oct 2012 : action tendency should be triggered by actually feeling the emotion, not only having the conditions
%               shouldn't be a demo rule but in pleiad_tendencies.pl
%               should be more generic : if the agent has a tendency to do a certain type of action
%                                        and knows an action of that type
%                                        then adopts the intention to perform that action
%                 (no reference to any specific emotion or tendency should appear here)

rule(15,T,Agt,
	% premises
	([emotion(Agt,anger(J),_P,D,T),
	NDT is 3*D,ND is NDT/4]),
	% conclusion
	intend(Agt,attack(J),ND,T,tendency)).

rule(16,T,Agt,
	% premises
	([emotion(Agt,fear,_P,D,T),
	NDT is 3*D,ND is NDT/4]),
	% conclusion
	intend(Agt,fuir,ND,T,tendency)).




%%%%%%%%%%%%%%%%%%
%% Commitment %%%%
%%%%%%%%%%%%%%%%%%

% NEW Nov2012
% committed(Agt1,Agt2,Prop) : Agt1 is committed towards Agt2 (possibly towards Agt1 itself) about Prop
% TODO : add action commitment
% modif: desires instead of goals (no goals here)

% TODO NOV2012 : change this rule
% the agent should commit only on the highest priority desire that was not abandoned before (not unreachable)
% with this rule Agt commits on all desires if possible
%rule(17,T,Agt,
%	% preconds
%		(
%		%% Agt desires to be responsible of Goal
%		desire(Agt,resp(Agt,Goal,_T),_Deg),
%		% Agt knows a plan to achieve this goal
%		bel(Agt, _Prem -> after(Agt,_Action,Goal),_D3,_Time,planification),
%		%% Agt is not committed on any goal yet
%		findall(Goal2,believe(Agt,committed(Agt,_,Goal2),_D,T),ListOfGoal),
%		forall(member(GoalA,ListOfGoal), believe(Agt,resp(Agt,GoalA,_T2),_D1,T)),
%		not(intend(Agt,_,_,T,global))
%		),
%	% effects
%		bel(Agt,committed(Agt,Agt,Goal),1,T,conviction)
%).

rule(17,T,Agt,
	% preconds
	([
		% findall desires not abandoned yet 
		findall(DegDes,
			(desire(Agt,P,DegDes),not(abandoned(Agt,P,T))), 
			LDeg),
		% sort by (increasing) degree
		sort(LDeg,ST),
		% get highest degree
		length(ST,N),N>0,nth1(N,ST,MyDeg),
		% get desire of that degree (most intense one)
		desire(Agt,Prop,MyDeg)
	]),
	% effects: commitment on that proposition
	committed(Agt,Prop,MyDeg,T)
).


%%%%%%%%%%%%%%%%%%%%
%% RESPONSIBILITY %%
%%%%%%%%%%%%%%%%%%%%

% NEW from Jeremy inference_rule.pl Nov2012
% if an agent believes that another agents wants him to be responsible for a Prop
% then it also adopts the belief that the other agent wants Prop to be true (with same source)
rule(18,T,Agt,
	% preconditions
	([
        % Agt believes that the Other has a goal that he is responsible for Prop
	believe(Agt,goal(Other,resp(Agt,Prop,_T1,Src),Deg,Time),_D1,T)
	]),
	% conclusion = Agt deduces that the Other has a goal that Prop be true
	bel(Agt,goal(Other,Prop,Deg,Time,Src),1,T,deduction)
).
			

%% rules about trust moved to pleiad_language_jeremy.pl



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ACTIONS PERFORMANCE : EFFECTS DEDUCTION %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%
%% direct consequences %
%%%%%%%%%%%%%%%%%%%%%%%%

%% imported from rule_csq predicate in Jeremy's inference_rule.pl file (Nov2012)

%% direct effects of non-speech act actions (conditional)
rule(221,T,Agt,
	%% premises
	([
	% if action was just done
	believe(Agt,done(Qqun,Action,_T1),DegBel1,T),
	%not a speech Act
	not(speaker(Action,Qqun)),
	% and action was believed to have consequences
	believe(Agt,Prem -> after(Qqun,Action,Csq),DegBel2,0),
	% premises are true
	believe(Agt,Prem,_DegBel3,T),
	% combine degs
	DegBel is DegBel1*DegBel2
	]),
	%% conclusion
	% then this consequence is now believed to be true
	bel(Agt,Csq,DegBel,T,deduction)
).


%% direct effects of non-speech act actions (not conditional)
% new July2014 in case no premise to imply the after effect of action
rule(222,T,Agt,
	%% premises
	([
	% if action was just done
	believe(Agt,done(Qqun,Action,_T1),DegBel1,T),
	%not a speech Act
	not(speaker(Action,Qqun)),
	% and action was believed to have consequences
	believe(Agt,after(Qqun,Action,Csq),DegBel2,0),
	% combine degs
	DegBel is DegBel1*DegBel2
	]),
	%% conclusion
	% then this consequence is now believed to be true
	bel(Agt,Csq,DegBel,T,deduction)
).

%% rule for direct effects of speech acts
% new july 2013 : check the belief does not exist yet so the rule cannot apply several times
rule(223,T,Agt,
     %% premises
     ([
        % speech act was just done
        believe(Agt,done(Qqun,SpeechAct,_T1),DegBel1,T),
	% and was believed to have some expressed effect
        believe(Agt,after(Qqun,SpeechAct,Csq),DegBel2,0),
        % not bel yet
        not(bel(Agt,Csq,_Anydeg,T,_Anysrc)),
        % combine degs
	DegBel is DegBel1*DegBel2
     ]),
     % conclusion
        % then this consequence is now believed to be true
        % Warning TODO Nov2012 : need to differentiate physical and epistemic effects of actions
	bel(Agt,Csq,DegBel,T,deduction)
). 
      
      
    
%% Rule for action consequences (conditional)
%% new July2014 - imported from jeremy rule_csq
rule(224,T,Agt,
        %% premises
	([
	% if action was done
	believe(Agt,done(Qqun,Action,_T1),DegBel1,T),
	%not a speech Act
	not(speaker(Action,Qqun)),
	% and action was believed to have consequences
	believe(Agt,Prem -> after(Qqun,Action,Csq),DegBel2,0),
	% premises are true
	believe(Agt,Prem,_DegBel3,T),
	% combine degs
	DegBel is DegBel1*DegBel2
        ]),
	% then this consequence is now believed to be true
	bel(Agt,Csq,DegBel,T,deduction)
).

%% Rule for action consequences (NON conditional)
%% new July2014 - imported from jeremy rule_csq
rule(225,T,Agt,
        %% premises
	([
	% if action was done
	believe(Agt,done(Qqun,Action,_T1),DegBel1,T),
	%not a speech Act
	not(speaker(Action,Qqun)),
	% and action was believed to have consequences
	believe(Agt,after(Qqun,Action,Csq),DegBel2,0),
	% combine degs
	DegBel is DegBel1*DegBel2
        ]),
	% then this consequence is now believed to be true
	bel(Agt,Csq,DegBel,T,deduction)
).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% rules for indirect effects of actions %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

rule(23,Time,Agt,
	%% CONDITIONS
	([
	bel(Agt,after(Qqun,Action,EffetDirect),DegBel,Time,_),
	bel(Agt,EffetDirect->EffetIndirect,DegImpl,Time,_),
	Deg is DegBel*DegImpl
	]),
	%% EFFECTS
	(
	bel(Agt,after(Qqun,Action,EffetIndirect),Deg,Time,deduction)
	)).








%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% user's goals adoption
%% based on trust
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  rule(25,
  % preconds
  ([
  instant(T),
  believe(Agt,goal(Qqun,resp(Agt,P,_T),DGoal),_DegBel,T),
  P \= believe(Agt,_P3,_D3,_T3),
  not(believe(Agt,P,_D2,T)),
  trust(Agt,Qqun)
  ]),
  % effects
  (
  goal(Agt,resp(Agt,P,_T1),DGoal)
  )).
	








    
%%%%%%%%%%
%% NEW APRIL 2013
%% rules for logic of ground operator
%%%%%%%%%%

% todo : logic of grounded in pleiad_demo
% pour pouvoir deduire de axiome 2 le grounded(K,illoc point) prouve sur papier
