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

% created November 2012
% to centralise all predicates for assertions and retractation
% of mental attitudes in agents KB
% plus abbreviations for mental attitudes
% plus vectors of mental attitudes


%% TODO MAY 2013
%% add knowval cf PLEID Jeremy / Moteur de raisonnement/*.pl
%% and PLEIAD Jeremy/Moteur de raisonnement/kbs/holiday_bc.pl
%% knowval(Agt,Prop,Val)
%% abbreviation for believe(Agt,Prop=Val,_Deg,T) ?

%% new MAY 2013 : assert bel handles not(not(P)) in prop content
%% new June 2013: goal has a source

% new June 2014 : intentions have a source/type. Kept old assert_intention for retro-compatibility

%%%%%%%%%%
%% CONTENT of KBMGR
%%   * assertion of mental attitudes (conflict management)
%%   * respective priorities of various sources for mental attitudes
%%   * retractation of mental attitudes (no fail)
%%   * searching mental attitudes (return -sorted- vectors for each type)
%%   * abbreviation for some mental attitudes (believe, prob)
%%   * NEW june 2014 - checking validity of mental attitudes (taking care of unifying different degrees)
%%%%%%%%%%


%:- dynamic(believe/4).

%%%%%%%%%%%%%%%%%%%%%%
%% 1 - ASSERT       %%
%%%%%%%%%%%%%%%%%%%%%%
%% ASSERTIONS IN KB %%
%%  special cases   %%
%%%%%%%%%%%%%%%%%%%%%%

%% abbreviations to assert a new mental attitude in KB
% after retracting previous version of the same mental attitude from KB
% inspired from my version for beliefs (cf pleiad_beliefs.pl)

% copied from Jeremy's belief_manager.pl November 2012
% commented November 2012 (no goal in my semantics)
% added goals MAY 2013




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%   ASSERTIONS OF (NEW) BELIEFS   %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% special cases from Jeremy's belief_manager.pl (November 2012)
% to handle the beliefs about mental attitudes asserted by some demo rules

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% assert belief : special cases %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% special case 1a: belief about the agent's own belief (believe predicate)
assert_belief(Agt,believe(Agt,Prop,Deg,Time),DegBel,_TimeBel,SrcBel) :-
	NewDeg is Deg * DegBel,
	assert_belief(Agt,Prop,NewDeg,Time,SrcBel).  % previously Deg

%% special case 1b: belief about the agent's own belief (bel predicate)
assert_belief(Agt,bel(Agt,Prop,Deg,Time,Src), DegBel,_TimeBel, _SrcBel) :-
	NewDeg is Deg * DegBel,
	assert_belief(Agt,Prop,NewDeg,Time,Src).  % previously Deg
	
%% special case 2: belief about the agent's own goal
% commented November 2012 (no goal in my semantics) + Rk: there should be an instant in the goal
% MAY 2013 : uncommented + added instant to goal
assert_belief(Agt,goal(Agt,Prop,Deg,Time,Src),DegBel,_TimeBel,_SrcBel) :-
	NewDeg is Deg * DegBel,
	assert_goal(Agt,Prop,NewDeg,Time,Src).
	
%% special case 3: belief about the agent's own intentions
assert_belief(Agt,intend(Agt,Prop,Deg,Time,Src),DegBel,_TimeBel,_SrcBel) :-
	NewDeg is Deg * DegBel,
	assert_intention(Agt,Prop,NewDeg,Time,Src).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% general case                       %%
%% belief about something that is NOT %%
%% a mental attitude of the agent     %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% special case of double negation
assert_belief(Agt,not(not(Prop)),Deg,Time,Src) :-
    assert_belief(Agt,Prop,Deg,Time,Src).


% assert a belief consists in managing conflicts with already existing beliefs (in particular priority due to source)
assert_belief(Agt,Prop,Deg,Time,SrcNew) :- 
            % find previous beliefs
            find_existing_beliefs(Agt,Prop,Time,[]),
            %% if the list L is empty: do nothing (in terms of retracting things)
            %% and directly assert the new belief
            kassert(bel(Agt,Prop,Deg,Time,SrcNew)),!.


% if there were previous beliefs about P or not(P)
assert_belief(Agt,Prop,Deg,Time,SrcNew) :-
            % find previous beliefs
            find_existing_beliefs(Agt,Prop,Time,L),
            % this list is not empty
            member(_,L),
	    % for all beliefs in the list
	    forall((  member(bel(A,P,D,T,SrcOld),L),
	    % whose source is less prio than that of the new belief
			mostprio(SrcNew,SrcOld)),
			% retract the old less prioritary belief
			( retract(bel(A,P,D,T,SrcOld)),
			% and assert the new most prioritary one instead
			kassert(bel(Agt,Prop,Deg,Time,SrcNew))  
			%% if there is a double to erase, still assert the new belief only once
	    )),!. % end forall

% build the list of existing beliefs about the same prop (or its negation) at the same time
% by the same agent from any source and of any degree
find_existing_beliefs(Agt,Prop,Time,List) :-
            %% find another belief about P or not(P) and extract its source
            % LP = list of beliefs about P
	    findall(bel(Agt,Prop,Deg1,Time,Src1),bel(Agt,Prop,Deg1,Time,Src1),LP),
	    % LNP = list of beliefs about negation(P) 
	    % (ie about not(P) if P atomic, or about Prop if P is not(Prop))   
	    findall(bel(Agt,NProp,Deg2,Time,Src2),(bel(Agt,NProp,Deg2,Time,Src2),(Prop=not(NProp);NProp=not(Prop))),LNP),
	    % L = union of the two lists LP and LNP
	    merge(LP,LNP,List).


                
                				 

% assert a new belief = assert the belief (managing priority and conflicts), then put focus on this new belief
assert_new_belief(Agt,Prop,Deg,Time,Src) :- assert_belief(Agt,Prop,Deg,Time,Src),set_focus(Agt,Prop,1,Time).


%%%%
%% DESIRES / UNDESIRES
%%%%


% for other mental attitudes
assert_desire(Agt,Prop,Deg) :- 
	ifpossible(retract(desire(Agt,Prop,_)),
	assert(desire(Agt,Prop,Deg))).

assert_undesire(Agt,Prop,Deg) :- 
	ifpossible(retract(undesire(Agt,Prop,_)),
	assert(undesire(Agt,Prop,Deg))).






%%%%
%% GOALS
%%%%
% uncommented MAY 2013
% should not backtrack (goal asserted twice, once after retracting, once directly)

% assert goal/notgoal
assert_goal(Agt,Prop,Deg,Time,Src) :- 
	ifpossible(retract(goal(Agt,Prop,_,_,_)),
	assert(goal(Agt,Prop,Deg,Time,Src))),!.
assert_notgoal(Agt,Prop,Deg,Time,Src) :- 
	ifpossible(retract(notgoal(Agt,Prop,_,_,_)),
	assert(notgoal(Agt,Prop,Deg,Time,Src))),!.

% assert conditional goal
assert_condgoal(Agt,Cond,Prop,Deg,Time,Src) :- 
	ifpossible(retract(condgoal(Agt,Cond,Prop,_,_,_)),
	assert(condgoal(Agt,Cond,Prop,Deg,Time,Src))),!.


condgoal(Agt,true,Prop,Deg,Time,Src) :- goal(Agt,Prop,Deg,Time,Src),!.


%%%%
%% INTENTIONS 
%%%%

%% assert intention (after trying to retract previous one)
% retro compat
assert_intention(Agt,Prop,Deg,Time) :-
        assert_intention(Agt,Prop,Deg,Time,nosource).

% new June 2014 - assert intention with 5 params : +source/type
assert_intention(Agt,Prop,Deg,Time,Src) :-
        %ground(Prop),
        % can have the same intention from a different source: keep both
	nofail(retract(intend(Agt,Prop,_,Time,Src))),
	assert(intend(Agt,Prop,Deg,Time,Src)),
        print('New intend: '+ Prop+Deg+Src+'\n').

% new june 2014 allow intentions with content not (completely) grounded
%assert_intention(Agt,Prop,Deg,_Time,_Src) :-
%    print('cannot assert intention with content not grounded : '+Agt+Prop+Deg).

retract_intention(Agt,Prop) :-
    nofail(retract(intend(Agt,Prop,_Deg,_Time,_Src))).






%%%%%%%%%%%%%%%%%%
% 2 - PRIORITIES %
%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%
%%%     PRIORITIES    %%%
%%% source of beliefs %%%
%%%%%%%%%%%%%%%%%%%%%%%%%

% observation: from stimulus 
% conviction: from the agent's initial KB
% communication: from speech act's effects (speech_act.pl)
% deduction: from inference rules

%auxiliary function to determine respective priority
% depending on source of belief
% obs > convic > comm > deduc
mostprio(observation,conviction).
mostprio(observation,communication).
mostprio(observation,deduction).
mostprio(conviction,communication).
mostprio(conviction,deduction).
mostprio(communication,deduction).

% if both belief come from the same source
% the newest one has priority ove the old one
% mostprio(observation,observation).
% mostprio(conviction,conviction).
% mostprio(communication,communication).
% mostprio(deduction,deduction).
mostprio(P,P).

% NEW
% speech_rule: speech act's preconditions and effects (speech_act.pl)
% so that they can be excluded from reasoning in pleiad_demo.pl rules
% mostprio(speech_rule,speech_rule).
mostprio(communication,speech_rule).
mostprio(observation,speech_rule).
mostprio(conviction,speech_rule).
mostprio(deduction,speech_rule).

% planification: specific plans in agent's KB
% planning source of beliefs (beliefs about consequences of actions)
% cf bel_action in pleiad_action.pl
% mostprio(planification,planification).
mostprio(speech_rule,planification).
mostprio(communication,planification).
mostprio(observation,planification).
mostprio(conviction,planification).
mostprio(deduction,planification).


% RK MAY 2013 : trust source of belief = communication?


%%%%%%%%%%%%%%%%%
%% 3 - RETRACT %%
%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%
%% 4 - SEARCH   %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% RETURN VECTORS OF MENTAL ATTITUDES %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% to be used in pleiad_demo to determine if a mental attitude deduced is new
vecteur_bdi(Agt,VBDI) :-
    instant(T),
    vecteur_BDI_time(Agt,T,VBDI).
    
vecteur_BDI_time(Agt,T,VBDI) :-    
    vecteur_belief(Agt,T,VBel),
    vecteur_desire(Agt,VDes),
    vecteur_goal(Agt,T,VGoal),
    vecteur_intend(Agt,T,VInt),
    merge(VBel,VDes,V1),
    merge(V1,VGoal,V2),
    merge(V2,VInt,VBDI).

% vecteur of new BDI at current instant (compared to T-1)
% Must relax time for comparison... for each mental attitude only keep new ones, then merge all
vecteur_new_BDI(Agt,VNBDI) :-
    instant(T),
    T1 is T-1,
    % new beliefs
    findall(bel(Agt,BProp,BDeg,T,BSrc),(bel(Agt,BProp,BDeg,T,BSrc),not(bel(Agt,BProp,_AnyBDeg,T1,_AnyBSrc))),VNBel),
    % new goals (no new desires since no time explicit in them)
    findall(goal(Agt,GProp,GDeg,T,GSrc),(goal(Agt,GProp,GDeg,T,GSrc),not(goal(Agt,GProp,_AnyGDeg,T1,_AnyGSrc))),VNGoal),
    % new intentions
    findall(intend(Agt,IProp,IDeg,T,ISrc),(intend(Agt,IProp,IDeg,T,ISrc),not(intend(Agt,IProp,_AnyIDeg,T1,_AnyISrc))),VNInt),
    % merge all new mental attitudes
    merge(VNBel,VNGoal,V1),
    merge(V1,VNInt,VNBDI).



%%%%%%%
%% Vector of emotions
%%%%%%%

% cf pleiad_emotions.pl



%%%%%%%
%% Vector of intentions
%% to be used in planning
%%%%%%%

% moved here from pleiad_tools.pl October 2012
vecteur_intend(Agt,T,Vintend_sorted) :- 
	findall(intend(Agt,P,D,T,S),intend(Agt,P,D,T,S),V),
	sort_intend(V,Vintend_sorted).

%%%%%%%%
%% vector of goals
%% new june 2013
%%%%%%%%

vecteur_goal(Agt,T,Vgoals) :-
    findall(goal(Agt,P,D,T,S),goal(Agt,P,D,T,S),V),
    sort_goal(V,Vgoals).


%%%%%%%%%%
%% vector of desires
%% moved from pleiad_desires June 2013
%%%%%%%%%%

% vector of desires sorted by intensity at a given instant
vecteur_desire(A,L) :- findall(desire(A,P,D),desire(A,P,D),L1),
			findall(undesire(A,P,D),undesire(A,P,D),L2),
			merge(L1,L2,LPS),
			sort_des(LPS,L).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% EXTRACTION OF BELIEFS %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%
% ALL BELIEFS
%%%%%%%%

% vector beliefs ordered by focus at a given instant (sorting function in pleiad_tools.pl)
vecteur_belief(A,T,L) :- 
	findall(bel(A,P,D,T,S),
	bel(A,P,D,T,S),LPS),
	sort_bel(LPS,L).


%%%%%%%%
% BELIEFS ABOUT IDEALS
%%%%%%%%

% vector of beliefs about the agent's ideals and unideals
vecteur_ideal(A,T,L) :- 
			% get all beliefs
			vecteur_belief(A,T,Lbel),
			% extract list of beliefs about ideals
			findall(bel(A,P,D,T,S),
				(member(bel(A,P,D,T,S),Lbel),P=ideal(_,_,_)),
				L1),
			% extract list of beliefs about unideals
			findall(bel(A,P,D,T,S),
				(member(bel(A,P,D,T,S),Lbel),P=unideal(_,_,_)),
				L2),
			% merge these 2 lists to obtain list of normative beliefs
			merge(L1,L2,L).


%%%%%%%%
% BELIEFS ABOUT EXPECTATIONS
%%%%%%%%

vecteur_expect(A,T,L) :- vecteur_belief(A,T,Lbel),
						findall(bel(A,P,D,T,S),
								(member(bel(A,P,D,T,S),Lbel),P=expected(_,_,_)),
								L1),
						findall(bel(A,P,D,T,S),
								(member(bel(A,P,D,T,S),Lbel),P=unexpected(_,_,_)),
								L2),
						merge(L1,L2,L).

%%%%%%%%
% PURE BELIEFS
%%%%%%%%

vecteur_purebel(A,T,L) :- vecteur_belief(A,T,Lbel),
						vecteur_ideal(A,T,Lid),
						vecteur_expect(A,T,Lex),
						findall(Bel,(member(Bel,Lbel),not(member(Bel,Lid)),not(member(Bel,Lex))),L).

% en cours October 2012
%vecteur_purebel(A,T,L) :- vecteur_belief(A,T,Lbel),
%			  findall(Bel,(member(Bel,Lbel),Bel\=ideal(_,_,_),Bel\=...),L).


%%%%%%%%%
% SPLIT in one single function
%%%%%%%%%

vector_bel_split(A,T,LBel,Lidl,Lexp,Lpure) :- 
	% all beliefs
	vecteur_belief(A,T,LBel),
	% about ideals and unideals
	findall(bel(A,P,D,T,S),(member(bel(A,P,D,T,S),Lbel),P=expected(_,_,_)),L1),
	findall(bel(A,P,D,T,S),(member(bel(A,P,D,T,S),Lbel),P=unexpected(_,_,_)),L2),
	merge(L1,L2,Lidl),
	% about expectations
	findall(bel(A,P,D,T,S),(member(bel(A,P,D,T,S),Lbel),P=expected(_,_,_)),L3),
	findall(bel(A,P,D,T,S),(member(bel(A,P,D,T,S),Lbel),P=unexpected(_,_,_)),L4),
	merge(L3,L4,Lexp),
	% pure bels
	findall(Bel,(member(Bel,LBel),not(member(Bel,Lidl)),not(member(Bel,Lexp))),Lpure).








%%%%%%%%%%%%%%%%%%%%%%%
%% 5 - ABBREVIATIONS %%
%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% logical theory  %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%
% check that a proposition is a mental attitude of a given agent
% to be completed with every new mental attitude
% abbreviation for Jeremy is_own_mental_attitude
%%%

mental_attitude(A,bel(A,_,_,_,_)).
mental_attitude(A,desire(A,_,_)).
mental_attitude(A,undesire(A,_,_)).
mental_attitude(A,believe(A,_,_,_)).
mental_attitude(A,prob(A,_,_,_)).
mental_attitude(A,intend(A,_,_,_,_)).
mental_attitude(A,goal(A,_Prop,_Deg,_Time,_Src)).
mental_attitude(A,condgoal(A,_Cond,_Prop,_Deg,_Time,_Src)).
mental_attitude(A,emotion(A,_,_,_,_)).
% new July 2013 - knowval and knowif abbreviations
mental_attitude(A,knowval(A,_Prop,_T)).
mental_attitude(A,knowif(A,_Prop,_T)).
% grounding at the very end because backtrack is infinite - max size of group to prevent this
% FIXME JULY2013 out of global stack when backtracking again
mental_attitude(A,grounded(Group,_Prop)) :- ground(Group),length(Group,L), L<7, member(A,Group).

mental_attitude(A,notgoal(A,_P,_D,_T,_S)).

% nested beliefs
%is_own_mental_state(A,bel(A,believe(A,P,D,T),_,_,_)) :- is_own_mental_state(A,bel(A,P,D,T,_)),!.
%is_own_mental_state(A,bel(A,prob(A,P,D,T),_,_,_)) :- is_own_mental_state(A,bel(A,P,D,T,_)),!.
%is_own_mental_state(A,bel(A,bel_prob(A,P,D,T),_,_,_)) :- is_own_mental_state(A,bel(A,P,D,T,_)),!.
%is_own_mental_state(A,bel(A,P,_,_,_)) :- ((mental_state(P),is_own_mental_state(A,P));true),!.


%%%%%%%%%%%%%%%%%
%% establishing abbreviations in conditions %%
%%%%%%%%%

% an abbreviation holds - abbrv must be instantiated, predicate will instantiate its defining MA
% used if goal is an abbrv while asserted effect of speech act is a mental attitude
% used if goal is a mental attitude (bel) while asserted effect of speech act is an abbrv (eg believe)
% is_validated_by(MentalAttitude,AbbrvMA) :-

% Now with validates_direct and validates_indirect that allows only one intermediate MA to avoid infinite loops
% caused by circular definitions where MA1 is in the body of a clause with MA2 as head, and vice versa too (ex: bel-believe, bel-not-not, etc)
% FIXME voir si le validates_direct peut etre recursif sans tout planter

validates_direct(Abbrv,MA) :-
    mental_attitude(Agt,Abbrv),mental_attitude(Agt,MA),
    % prevent error "arguments not sufficiently instantiated" BUT allow partially instantiated arguments?
    catch(clause(Abbrv,Body),error(instantiation_error, context(system:clause/2, _G1417)),(display_message(['caught instantiation error']),fail)),
    clauses_to_list(Body,BL),
    member(MA,BL),!.

% if fails because Abbrv not instantiated, turn it around, MA might/should be instantiated
% no recursive call (%validates2(MA,Abbrv),!.) - out of local stack
validates_direct(Abbrv,MA) :- 
    mental_attitude(Agt,Abbrv),mental_attitude(Agt,MA),
    % prevent error "arguments not sufficiently instantiated" BUT allow partially instantiated arguments?
    catch(clause(MA,Body),error(instantiation_error, context(system:clause/2, _G1417)),(display_message(['caught instantiation error 2']),fail)),
    clauses_to_list(Body,BL),
    member(Abbrv,BL),!.

% indirect validation via one other MA only (prevents loops) - symmetric because validates_direct is symmetric
validates_indirect(MA1,MA2) :-
        validates_direct(MA1,MA3),validates_direct(MA3,MA2).

% case 1 : MA1 is an abbreviation of MA2
validates2(MA1,MA2) :-
    validates_direct(MA1,MA2).

% case 2 : MA2 is an abbreviation of MA1
validates2(MA1,MA2) :-
    validates_direct(MA2,MA1).

% case 3 : MA1 and MA2 are abbreviations of the same mental attitude
validates2(MA1,MA2) :-
    validates_indirect(MA1,MA2).


% validated beliefs can be in different forms (establish finds all abbreviations)
% build an extended vector of all beliefs that hold in any form = vecteur_belief + beliefs established by these
vecteur_allbels(Agt,Time,V) :-
    findall(ExtBel,(vecteur_belief(Agt,Time,L),member(Bel,L),validates2(Bel,ExtBel)),V).

% now a valid belief is one that is in this extended vector that contains all beliefs and all their reformulations
valid_belief(Agt,Time,Prop) :-
    vecteur_allbels(Agt,Time,V),
    member(bel(Agt,Prop,_Deg,Time,_Src),V),!.

%%%
% abbreviations
%%%
% the 5th parameter of bel (= source) is transparent for believe
believe(A,Prop,D,T) :- bel(A,Prop,D,T,_).   % ,D>0.5.
%% WARNING: any bel even if weak is now a belief, otherwise goals are not considered reached when degree is too low...
%%          but out of global stack oif removes check on degree 
prob(A,Prop,D,T) :- bel(A,Prop,D,T,_),D>0, D=<0.5.   %% even if tiny hope, we want hope

% A knows the value of Prop at instant Time if at that instant it has a belief about a grounded value of Prop
knowval(A,Prop,Time) :- bel(A,value(Prop,Val),_Deg,Time,_Src), ground(Val).
% A knows IF a Prop is true if it has a belief about Prop or its negation not(Prop)
knowif(A,not(Prop),Time) :- bel(A,Prop,_Deg,Time,_Src).
knowif(A,not(Prop),Time) :- bel(A,not(Prop),_Deg,Time,_Src).
knowif(A,Prop,Time) :- bel(A,Prop,_Deg,Time,_Src).
knowif(A,Prop,Time) :- bel(A,not(Prop),_Deg,Time,_Src).

% Jeremy's bel_prob (logi_axioms.pl) - a supprimer pour simplifier (c'est juste un bel)
% bel_prob(A,Prop,D,T) :- bel(A,Prop,D,T,_Src).
% Jeremy's know_about a supprimer (c juste un knowif)
know_about(A,P,T) :- (bel(A,P,_D,T,_S);bel(A,not(P),_D1,T,_S1)).


% inside beliefs (in case not grounded so awareness of MA rule does not apply)
bel(Agt,knowval(Agt,P,Time),1,Time,conviction) :- instant(Time),ground(Agt),ground(P),knowval(Agt,P,Time).
bel(Agt,knowif(Agt,P,Time),1,Time,conviction) :- instant(Time),ground(Agt),ground(P),knowif(Agt,P,Time).


%%%%
%% imbricated negations
%%%%

% infinite loop with validates2 - 21july2014
bel(Agt,not(not(P)),Deg,Time,Src) :- ground(P),bel(Agt,P,Deg,Time,Src),!.


%%%%
%% awareness of mental attitudes - NEW Nov2012
%%%%

% an agent automatically believes with max degree of conviction
% any proposition that is one of its own mental attitudes
bel(Agt,P,1,T,conviction) :- instant(T),ground(P),mental_attitude(Agt,P),P.

% new APRIL 2013: must also check that P holds... an agent is only aware of mental attitudes it really has
%                 warning: only check that P holds AFTER checking it is a mental attitude (or it could have no sense and create an error)
% WARNING : ground(P) is essential to prevent infinite loops ! (tested JULY 2013)

%%%%
%% awareness of performance of expressive speech acts
%%%%

% the agent believes that an expressive was done with some (grounded!) degree
% as soon as it believes that the same expressive was done with any higher degree
% ie the same emotion was already expressed with a higher degree (since intensity naturally decays over time)
% (but if the intensity increases, then the emotion can be expressed again)
bel(Agt,done(Someone,express(Someone,Someother,[Emo,Object,DegEmo]),Sometime),DegBel,TimeBel,SrcBel) :-
        ground(DegEmo),
        bel(Agt,done(Someone,express(Someone,Someother,[Emo,Object,AnyDeg]),Sometime),DegBel,TimeBel,SrcBel),
        AnyDeg>=DegEmo.

%%%
% awareness of current time
% at time T, any agent believes that current time is T with degree 1
%%%
bel(_,instant(T),1,T,observation) :- instant(T).


%%%
% beliefs about actions
% cf pleiad_action.pl
%%%

bel(A,P,D,T,planification) :- bel_action(A,P,D,T).

%%%
% ideals and unideals
%%%

% from Jeremy's inference_rule.pl
% transformed into an abbreviation?
% should we (logically / semantically) have unideal as the exact opposite of ideal ?

% agents aware of their own ideals/unideals
bel(B,unideal(B,P,D),_DD,_TT,_SS) :- ground(P),unideal(B,P,D).
bel(B,ideal(B,P,D),_DD,_TT,_SS) :- ground(P),ideal(B,P,D).

% the opposite of an ideal is unideal
bel(A,unideal(B,P,D),DD,TT,SS) :- ground(P),P \= not(_),bel(A,ideal(B,not(P),D),DD,TT,SS).
bel(A,unideal(B,not(P),D),DD,TT,SS) :- ground(P),bel(A,ideal(B,P,D),DD,TT,SS).

%%%%
% goals and notgoals
% from Jeremy's inference_rule.pl
%%%%

notgoal(A,P,D,T,S) :- ground(P),P \= not(_), goal(A,not(P),D,T,S).
notgoal(A,not(P),D,T,S) :- ground(P),goal(A,P,D,T,S).



%%%%%%%%%%%%%%%%%%%%%%%%
%% 6) CHECK VALIDITY  %%
%%%%%%%%%%%%%%%%%%%%%%%%

% check validity of mental attitude - filter to access the KB
% TODO such a filter should also be used for abbreviations (cf JSA KB filters)

% filter should list all specific cases
% check_kb(bel())

% default case : mental attitude is exactly the same in the KB



%%%%%%%%%%%%%%%%%%%%
%% match degrees  %%
%%%%%%%%%%%%%%%%%%%%

% a goal is achieved if intended degree (of belief generally) is reached exactly OR exceeded!!
% AND the other features are identical !
achieves_goal(believe(Agt,Prop,DG,T),believe(Agt,Prop,DB,T)) :- ground(DB),ground(DG),DB >= DG,!. 
    
% a goal is also achieved by abbreviations - validates2 should be symmetric
achieves_goal(MABel,MAGoal) :- validates2(MABel,MAGoal).
    
% if no degree, ie not a mental attitude: succeed
achieves_goal(P,P) :- !.    %ground(P),!.

%degree(bel(_,_,D,_,_),D) :- !.
%degree(believe(_,_,D,_),D) :- !.
%degree(goal(_,_,D,_,_),D) :- !.
%degree(intend(_,_,D,_,_),D) :- !.

%achieves_goal(goal(_,Prop1,_,_,_),Prop2) :-
%    degree(Prop1,D1),
%    degree(Prop2,D2),
%    D2 >= D1,!.

%achieves_goal(GoalProp,BelProp) :-
%    degree(GoalProp,DG),
%    degree(BelProp,DB),
%    DB >= DG,!.
 
