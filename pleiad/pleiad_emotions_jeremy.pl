%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                                                       %%
%% 	PLEIAD Reasoning Engine				 %%
%%                                                       %%
%% Author : Jeremy Riviere, Carole Adam                  %%
%% Version : v1 - February 2012                          %%
%% Based on PLEIAD, developed by Carole Adam 2007        %%
%% Application to the CECIL project			 %%
%%							 %%
%%    ------- DO NOT DISTRIBUTE -------                  %%
%%                                                       %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% EMOTION INTERFACE %%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The emotion definition file must have the following predicates:
% feel_emotion - the felt emotion at the time T
% max_emotion_felt - return the felt emotion with the higher degree
% event_evaluation

:- dynamic(intrinsic_p/2).
:- dynamic(assert_Scherer/5).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% copied from main emotion file  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FIXME should be in a common emotion file , outside of separate emotiondef files

% returns list L of all emotions of agent A
vecteur_emo(A,T,L) :- findall(emotion(A,E,P,D,T),emotion(A,E,P,D,T),L).



%%%%%%%%%%%%%%%%%%%%%%%%
%% EmOtion ExprEssiOn %%
%%%%%%%%%%%%%%%%%%%%%%%%

%% new june2014 - to instantiate predicate defined in pleiad_emotions in normal version

express_emotion(A,emotion(A,E,[P],D,T),T) :- j_emotion(A,E,P,D,T), seuil_expression(A,Seuil), D>Seuil.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Scherer's checks %%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 event_evaluation(Agt,SpeechAct,T) :-   %% Last speech act consequences
				    saturer_bc_return(Agt,CsqList),
                                    debug_message(demo,['SaturerBC returns new consequences list = ',CsqList,' ']),
                                    novelty(Agt,SpeechAct,CsqList,T,DegNov),
				    forall(member(Csq,CsqList), s_assert(Csq)),
                                    intrinsic_p(SpeechAct,DegIntrinsic),
				    % BGR deduction %boucle_satur_BGR,
                                    saturer_bc(Agt),
                                    implications(Agt,T,DegImp),
                        	    cop_potential(Agt,SpeechAct,T,DegCop),
                                    standards(Agt,T,DegStand), 
                                    assert_Scherer(DegNov,DegIntrinsic,DegImp,DegCop,DegStand).
											  


%%%%
%% SCHERER
%%%%
				
assert_Scherer(Deg1,Deg2,Deg3,Deg4,Deg5) :- 
	ifpossible(retract(scherer_checks(_,_,_,_,_)),
	assert(scherer_checks(Deg1,Deg2,Deg3,Deg4,Deg5))).




%%%%%%%%% Relevance
%%% Novelty speech act outside of a dialogue schema, the user informs of something unknown / contradictory with existing beliefs

% Action case
novelty(_,action(_,_),_,_,0) :-!.

% First case: I'm the speaker
novelty(Agt,SpeechAct,_CsqList,_T,0) :- speaker(SpeechAct,Agt),!.

% Second case: I was expecting the speech act
novelty(Agt,SpeechAct,_CsqList,_T,0) :-  speaker(SpeechAct,Qqun),
							    Qqun \= Agt,
								illoc_force(SpeechAct,Force),
								expected(Agt,Qqun,Force),!.
								
% Second case: I wasn't expecting the speech act
% but this act is part of a dialogue scheme
novelty(Agt,SpeechAct,_CsqList,T,0.7) :-  speaker(SpeechAct,Qqun),
							    Qqun \= Agt,
								T1 is T-1,
								illoc_force(SpeechAct,Force),
								expected(Agt,Qqun,Force2),
								Force \= Force2,
								dialogue_rule(OldSpeechAct,SpeechAct,_),
								believe(Agt,done(Agt,OldSpeechAct,_T2),_D1,T1),!.
																
% Second case: I wasn't expecting the speech act
% and this act is not part of a dialogue scheme
novelty(Agt,SpeechAct,_CsqList,_T,1) :-  speaker(SpeechAct,Qqun),
							    Qqun \= Agt,
								illoc_force(SpeechAct,Force),
								expected(Agt,Qqun,Force2),
								Force \= Force2,
								% The act is not within a dialogue scheme
								not(dialogue_rule(_OldSpeechAct,SpeechAct,_)),!.
								
								
% Third case: I wasn't expecting anything
novelty(Agt,SpeechAct,CsqList,T,1) :- speaker(SpeechAct,Qqun),
							    Qqun \= Agt,
								T1 is T-1,
								not(expected(Agt,_,_)),
								% The act is not within a dialogue scheme
								not(dialogue_rule(_OldSpeechAct,SpeechAct,_)),	
								% And I believed not(Csq)
								member(Csq,CsqList),  believe(Agt,not(Csq),_D,T1),!.									
								
novelty(Agt,SpeechAct,CsqList,T,0.7) :- speaker(SpeechAct,Qqun),
							    Qqun \= Agt,
								T1 is T-1,
								not(expected(Agt,_,_)),
								% The act is within a dialogue scheme
								dialogue_rule(OldSpeechAct,SpeechAct,_),
								believe(Agt,done(Agt,OldSpeechAct,_T2),_D1,T1),
								% And I believed not(Csq)
								member(Csq,CsqList), believe(Agt,not(Csq),_D,T1),!.									
								
novelty(Agt,SpeechAct,CsqList,T,0.5) :- speaker(SpeechAct,Qqun),
							    Qqun \= Agt,
								T1 is T-1,
								not(expected(Agt,_,_)),
								% The act is not within a dialogue scheme
								not(dialogue_rule(_OldSpeechAct,SpeechAct,_)),	
								% And What is said is new for me
								member(Csq,CsqList), not(know_about(Agt,Csq,T1)),!.			
								
novelty(Agt,SpeechAct,CsqList,T,0.2) :- speaker(SpeechAct,Qqun),
							    Qqun \= Agt,
								T1 is T-1,
								not(expected(Agt,_,_)),
								% The act is not within a dialogue scheme
								not(dialogue_rule(_OldSpeechAct,SpeechAct,_)),								
								% And What is said is not new for me
								forall(member(Csq,CsqList), believe(Agt,Csq,_D,T1)),!.
								
novelty(Agt,SpeechAct,CsqList,T,0.2) :- speaker(SpeechAct,Qqun),
							    Qqun \= Agt,
								T1 is T-1,
								not(expected(Agt,_,_)),
								% The act is within a dialogue scheme
								dialogue_rule(OldSpeechAct,SpeechAct,_),
								believe(Agt,done(Agt,OldSpeechAct,_T2),_D1,T1),
								% And What is said is new for me
								member(Csq,CsqList), not(know_about(Agt,Csq,T1)),!.		

novelty(Agt,SpeechAct,CsqList,T,0) :- speaker(SpeechAct,Qqun),
							    Qqun \= Agt,
								T1 is T-1,
								not(expected(Agt,_,_)),
						      	% The act is within a dialogue scheme
								dialogue_rule(OldSpeechAct,SpeechAct,_),
								believe(Agt,done(Agt,OldSpeechAct,_T2),_D1,T1),
								% And What is said is not new for me
								forall(member(Csq,CsqList), believe(Agt,Csq,_D,T1)),!.

novelty(_,_,_,_,0).								

%%% Intrinsic Pleasantness
%% action %%
intrinsic_p(action(_,_),0.8).

%% Positives %%
intrinsic_p(accept_offer(_,_,_),0.5).
intrinsic_p(promise(_,_,_),0.5).
intrinsic_p(assure(_,_,_),0.6).
intrinsic_p(accept_demand(_,_,_),0.5).
intrinsic_p(offer(_,_,_),0.5).

intrinsic_p(greet(_,_,_),0.7).
intrinsic_p(approve(_,_,_),0.7).
intrinsic_p(rejoice(_,_,_),0.8).
intrinsic_p(be_very_pleased(_,_,_),1).
intrinsic_p(thank(_,_,_),0.8).
intrinsic_p(congratulate(_,_,_),1).
intrinsic_p(satisfy(_,_,_),0.8).
intrinsic_p(boast(_,_,_),1).
intrinsic_p(suscribe(_,_,_),0.8).
intrinsic_p(compliment(_,_,_),1).

%% Negatives %%
intrinsic_p(contradict(_,_,_),-0.5).
intrinsic_p(refuse(_,_,_),-0.5).
intrinsic_p(renounce(_,_,_),-0.5).
intrinsic_p(deny(_,_,_),-0.5).

intrinsic_p(cry(_,_,_),-0.7).
intrinsic_p(disapprove(_,_,_),-0.7).
intrinsic_p(regret(_,_,_),-0.8).
intrinsic_p(apologize(_,_,_),-0.8).
intrinsic_p(beg_pardon(_,_,_),-1).
intrinsic_p(complain(_,_,_),-0.8).
intrinsic_p(moan(_,_,_),-1).

%% Default case %%
intrinsic_p(_,0).


%%%%%%%%% Implications
%%% Goal attainment (Goal P) and Causal attribution (Resp P)
implications(Agt,T,Deg) :- j_emotion(Agt,E,_P,D,T),
						   goal_emotion(E,Dpn),
						   Deg is D * Dpn,! .
						   
%% Default case %%						   
implications(_Agt,_T,0).

goal_emotion(joy,1). 
goal_emotion(rejoicing,1).
goal_emotion(gratitude(_),1).
goal_emotion(sadness,-1).
goal_emotion(regret,-1).
goal_emotion(disappointment(_),-1). 

%%%%%%%%% Coping potential
%%% Power/control
cop_potential(Agt,SpeechAct,_T,1) :- speaker(SpeechAct,Qqun),
							    Qqun \= Agt,
								(illoc_force(SpeechAct,ask);illoc_force(SpeechAct,demand)),
								prop_content(SpeechAct,Prop),
								find_one_plan(Agt,Prop,_Action),!.
								
cop_potential(Agt,SpeechAct,_T,1) :- speaker(SpeechAct,Agt),
								prop_content(SpeechAct,Prop),
								find_one_plan(Agt,Prop,_Action),!.
								
cop_potential(Agt,SpeechAct,_T,-1) :- speaker(SpeechAct,Qqun),
							    Qqun \= Agt,
								(illoc_force(SpeechAct,ask);illoc_force(SpeechAct,demand)),
								prop_content(SpeechAct,Prop),
								not(find_one_plan(Agt,Prop,_Action)),!.								
								
cop_potential(Agt,SpeechAct,T,1) :- speaker(SpeechAct,Qqun),
							    Qqun \= Agt,
								illoc_force(SpeechAct,ask_val),
								prop_content(SpeechAct,Prop),
								believe(Agt,know_val(Agt,Prop),_D,T),!.
								
cop_potential(Agt,SpeechAct,T,-1) :- speaker(SpeechAct,Qqun),
							    Qqun \= Agt,
								illoc_force(SpeechAct,ask_val),
								prop_content(SpeechAct,Prop),
								not(believe(Agt,know_val(Agt,Prop),_D,T)),!.			

cop_potential(Agt,SpeechAct,_T,-1) :- speaker(SpeechAct,Qqun),
							    Qqun \= Agt,
								(illoc_force(SpeechAct,refuse);illoc_force(SpeechAct,renounce)),!.							
								
cop_potential(_Agt,_SpeechAct,_T,0).				
			

%%%%%%%%% Norm compatibility
%%% Internal Standards (Ideal P)
standards(Agt,T,Deg) :- j_emotion(Agt,E,_P,D,T),
						   ideal_emotion(E,Dpn),
						   Deg is D * Dpn,! .
						   
%% Default case %%						   
standards(_Agt,_T,0).						   

ideal_emotion(moral_satisfaction,1).
ideal_emotion(adhesion(_),1). 
ideal_emotion(approval,1).
ideal_emotion(disapproval,-1).
ideal_emotion(guilt,-1).
ideal_emotion(reproach(_),-1).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% COMPLEX EMOTIONS FROM CECIL %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% list of all 12 CECIL emotion types (ontology)
emotiontype(E) :- member(E,[rejoicing,gratitude,regret,disappointment,moral_satisfaction,adhesion,guilt,reproach,joy,sadness,approval,disapproval]),!.

% used in pleiad_intentions, rule to express most intense emotion
max_emotion_felt([feel_emotion(A,Emotion,P,Deg,T),feel_emotion(A,_Emotion2,_P2,Deg2,T)|List],feel_emotion(A,Emotion3,P3,Deg3,T)) :- Deg >= Deg2, max_emotion_felt([feel_emotion(A,Emotion,P,Deg,T)|List],feel_emotion(A,Emotion3,P3,Deg3,T)).
max_emotion_felt([feel_emotion(A,_Emotion,_P,Deg,T),feel_emotion(A,Emotion2,P2,Deg2,T)|List],feel_emotion(A,Emotion3,P3,Deg3,T)) :- Deg < Deg2, max_emotion_felt([feel_emotion(A,Emotion2,P2,Deg2,T)|List],feel_emotion(A,Emotion3,P3,Deg3,T)).
max_emotion_felt([feel_emotion(A,Emotion,P,Deg,T)],feel_emotion(A,Emotion,P,Deg,T)).

feel_emotion(A,Emotion,P,Deg,T) :- j_emotion(A,Emotion,P,Deg,T), seuil_ressenti(A,Seuil), Deg>Seuil.

% rejoicing
% j_emotion(A,rejoicing,P,D,T) :- goal(A,P,D), believe(A,resp(A,P,_T),_D2,T).
j_emotion(A,rejoicing,P,D,T) :- goal(A,P,D,T,_S), 
				believe(A,resp(A,done(A,P,_T1),T),_DBel,T).

% gratitude
j_emotion(A,gratitude(B),P,D,T) :- believe(A,resp(B,P,T),_D2,T), goal(A,P,D,T,_S), A \= B .											

% regret
% definition equivalente a: goal(A,not(P),D), believe(A,resp(A,P,T),_D2,T). 
%  mais pas le choix(??) a cause des not(not(P)) qui ne sannulent pas
j_emotion(A,regret,not(P),D,T) :- notgoal(A,not(P),D,T,_S), believe(A,resp(A,not(P),T),_D2,T).

% disappointment
j_emotion(A,disappointment(B),P,D,T) :- notgoal(A,P,D,T,_S), believe(A,resp(B,P,T),_D2,T), A \= B .

% moral satisfaction
j_emotion(A,moral_satisfaction,P,D,T) :- ideal(A,P,D), believe(A,resp(A,P,T),_D2,T).

% adhesion
j_emotion(A,adhesion(B),P,D,T) :- ideal(A,P,D), believe(A,resp(B,P,T),_D2,T), A \= B .

% guilt
j_emotion(A,guilt,P,D,T) :- unideal(A,P,D), believe(A,resp(A,P,T),_D2,T).

% reproach
j_emotion(A,reproach(B),P,D,T) :- unideal(A,P,D), believe(A,resp(B,P,T),_D2,T), A \= B .

%% "basic" complex emotions

% joy
j_emotion(A,joy,saw(A,B),D,T) :- goal(A,saw(A,B),D,T,_S), believe(A,saw(A,B),_D2,T), focus(A,saw(B,A),_,T).

%sadness
j_emotion(A,sadness,P,D,T) :- notgoal(A,P,D,T,_S), believe(A,P,_D2,T),focus(A,P,D,T).

%approval
j_emotion(A,approval,P,D,T) :- ideal(A,P,D), believe(A,P,_D2,T),focus(A,P,D,T).

%disapproval
j_emotion(A,disapproval,P,D,T) :- unideal(A,P,D), believe(A,P,_D2,T),focus(A,P,D,T).
