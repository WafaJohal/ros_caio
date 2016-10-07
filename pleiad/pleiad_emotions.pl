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

:- multifile(emotion/5).

:- dynamic(emotion/5).

% compile expression of emotion
:- [pleiad_expression].

%%%%%%%%%%%%%%%%%%%%%%%%
% emotional rules      %
%%%%%%%%%%%%%%%%%%%%%%%%

% allows to centralise all verifications to be done for all emotions
emotion(A,E,P,D,T) :-
    % APRIL2013% saturer_bc,
    cond_emotion(A,E,P,D,T),D>0. 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% triggering mental attitudes of an emotion  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

:- discontiguous(bel_other_emo/6).

triggering_mental_attitudes(A,E,P,D,T,LMA) :-
    % cf pleiad_tools
    clause_list(cond_emotion(A,E,P,D,T),List),
    % extract only mental attitudes
    % findall(C,(member(C,List),mental_attitude(A,C)),LMA).
    % contrarily to findall, include keeps the links between uninstantiated variables
    include(mental_attitude(_),List,LMA).

% triggering_other_conds(A,E,P,D,T,LOC) :-
%    clause_list(cond_emotion(A,E,P,D,T),List),
%    % extract only mental attitudes
%    findall(C,(member(C,List),not(mental_attitude(_Agt,C))),LOC).

% old_believe_other_emotions(A,B,E,P,DegEmo,T) :-
%    % A believes that B has all the triggering mental attitudes for this emotion
%    triggering_mental_attitudes(B,E,P,_D,T,LMA),
%    findall(Deg,(member(MA,LMA),believe(A,MA,Deg,T)),LDeg),
%    product_list_degrees(LDeg,DegEmo).        

% bel_other_emo_nodeg(A,B,E,P,D,T) :-
%    triggering_mental_attitudes(B,E,P,D,T,LMA),
%    % cannot use findall as loses the link between uninstantiated variables
%    % findall(believe(A,MA,_Deg,T),member(MA,LMA),LMB),
%    % valid(LMB).
%    believe_list(A,LMA,T).


% FIXME: this old version might not work for instantiating the degree?    
bel_other_emo(A,B,E,P,D,T) :-
    % get the triggering conditions of that emotion
    clause_list(cond_emotion(B,E,P,D,T),List),
    % extract only mental attitudes
    partition(mental_attitude(_),List,LMA,LOC),
    % check that agent A believes all props in list LMA
    believe_list(A,LMA,T),
    % switch agent - get conditions of emotion for the other agent
    maplist(switch_agent(B,A),LOC,LOCD),
    % check these conditions are valid (def in pleiad_reasoning.pl)
    valid(LOCD).

% % exclude focus predicate
%    select(infocus(_,_,_),LOC,LOCD),
% no because needed to instantiate a degree    

% auxiliary : believe_list: check that an agent believes a list of propositions
believe_list(_Agt,[],_T) :- !.
believe_list(Agt,[Prop|List],T) :-
    believe(Agt,Prop,_Deg,T),
    believe_list(Agt,List,T).

% TODO : believe_list_clauses : returns a list of beliefs to be checked
% with valid() predicate
% so that instantiations are propagated

% auxiliary : switch agent in the focus predicate in LOC
switch_agent(Aold,Anew,infocus(Aold,P,D,T),infocus(Anew,P,D,T)) :- !.
switch_agent(_,_,P,P).

    % findall(believe(A,C,D,T),(member(C,List),mental_attitude(B,C)),LMA),    
    % and the other conditions are true
    % RK: in particular the object should be in focus for A (and not believed to be in focus for B)
    % findall(C,(member(C,List),not(mental_attitude(_Agt,C))),LOC),
    % merge(LMA,LOC,LL),
    % forall(member(Cond,LL),Cond).




% new version that gets the degree !

% other solution
% predicate that parses conditions
% adds belief if it is a mental attitude
% reverse agents if not
% and returns the new list of clauses
clauses_other_emotion(A,B,E,P,D,T,ListC) :-
    clause_list(cond_emotion(B,E,P,D,T),List),
    maplist(clause_emotion_b2a(A,B,T),List,ListC).

% case 1 : mental attitude of B -> bel(A,MA)
% FIXME: problem of the time in OneClause, need not be the same as time in the belief around it
clause_emotion_b2a(A,B,T,OneClause,bel(A,OneClause,_D,T,_S)) :-
    mental_attitude(B,OneClause),!.
    
% case 2 : other condition -> exchange A and B
clause_emotion_b2a(A,B,_T,OneClause,NewClause) :-
    switch_agent(B,A,OneClause,NewClause),!.

% new version of bel_other_emo that propagates instantiations
bel_other_emo(A,B,E,P,D,T) :-
    clauses_other_emotion(A,B,E,P,D,T,List),
    valid(List).



%%%%%%%%%%%%%%%%%%%%%%%%%
%% vectors of emotions %%
%%%%%%%%%%%%%%%%%%%%%%%%%

% returns list L of all emotions of agent A
vecteur_emo(A,T,L) :- findall(emotion(A,E,P,D,T),emotion(A,E,P,D,T),L).

% returns the list V of negative emotions of agent A at time T
% using the negative(..) predicate defined at the bottom of this file
vector_emo_neg(A,T,V) :- 
	findall(emotion(A,E,P,D,T),(emotion(A,E,P,D,T),negative(E)),V).

% returns the list V of positive emotions of agent A at time T
% using the positive(..) predicate defined at the bottom of this file
vector_emo_pos(A,T,V) :- 
	findall(emotion(A,E,P,D,T),(emotion(A,E,P,D,T),positive(E)),V).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Feeling and expressing emotions - personality thresholds %%
%% from Jeremy - CECIL Inaffech                             %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% emo is felt if intensity above threshold
feel_emotion(A,Emotion,P,Deg,T) :- emotion(A,Emotion,P,Deg,T), seuil_ressenti(A,Seuil), Deg>Seuil.

%% vector of all felt emotions
vecteur_emo_feel(A,T,L) :- findall(feel_emotion(A,Emo,P,D,T),feel_emotion(A,Emo,P,D,T),L).

%% find felt emotion with maximal intensity, in a list of emotions provided as parameter - recursive aux
max_emotion_felt_aux([feel_emotion(A,Emotion,P,Deg,T),feel_emotion(A,_Emotion2,_P2,Deg2,T)|List],feel_emotion(A,Emotion3,P3,Deg3,T)) :- Deg >= Deg2, max_emotion_felt_aux([feel_emotion(A,Emotion,P,Deg,T)|List],feel_emotion(A,Emotion3,P3,Deg3,T)).
max_emotion_felt_aux([feel_emotion(A,_Emotion,_P,Deg,T),feel_emotion(A,Emotion2,P2,Deg2,T)|List],feel_emotion(A,Emotion3,P3,Deg3,T)) :- Deg < Deg2, max_emotion_felt_aux([feel_emotion(A,Emotion2,P2,Deg2,T)|List],feel_emotion(A,Emotion3,P3,Deg3,T)).
max_emotion_felt_aux([feel_emotion(A,Emotion,P,Deg,T)],feel_emotion(A,Emotion,P,Deg,T)).

%% find max felt emotion of an agent - new june2014
max_emotion_felt(Agent,Emo) :- instant(T),vecteur_emo_feel(Agent,T,L),max_emotion_felt_aux(L,Emo),!.



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% complete triggering conditions %%%%
%%%% for all emotions               %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%
%% EVENT-BASED BRANCH %%
%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%
% well-being %
%%%%%%%%%%%%%%

cond_emotion(A,joy,[P],D,T)     :-
						infocus(A,P,Deg,T),
						believe(A,P,D1,T),
						desire(A,P,D2),
						D is D1*D2*Deg.


cond_emotion(A,sadness,[P],D,T) :-
						infocus(A,P,Deg,T),
						believe(A,P,D1,T),
						undesire(A,P,D2),
						D is D1*D2*Deg.


% prospect-based
cond_emotion(A,hope,[P],D,T) :-
						infocus(A,P,Deg,T),
						prob(A,P,D1,T),
						desire(A,P,D2),
						D is D1*D2*Deg.

cond_emotion(A,fear,[P],D,T) :-
						infocus(A,P,Deg,T),
						prob(A,P,D1,T),
						undesire(A,P,D2),
						D is D1*D2*Deg.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% confirmation-based emotions %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% warning: must use predicate emotion(..) and not cond_emotion for past emotion
%% because conditions are not true anymore, so we are relying on memory of emotions triggered at T-1

% confirm
cond_emotion(A,fearconfirmed,[P],D,T) :-
						infocus(A,P,Deg,T),
						emotion(A,fear,[P],D1,T0), T is T0+1,
						believe(A,P,D2,T), D is D1*D2*Deg.

cond_emotion(A,satisfaction,[P],D,T)  :-
						infocus(A,P,Deg,T),
						emotion(A,hope,[P],D1,T0),
						T is T0+1,
						believe(A,P,D2,T),
						D is D1*D2*Deg.


% disconfirm
cond_emotion(A,relief,[P],D,T) :-
						infocus(A,P,Deg,T),
						emotion(A,fear,[not(P)],D1,T0),
						T is T0+1,
						believe(A,P,D2,T),
						D is D1*D2*Deg.

% obliged to do a 2nd case of disconfirm emotions for when the object is a negation because
% fear/hope are not about not(P) but about not(not(P)), which Prolog does NOT reduce to P...

cond_emotion(A,relief,[not(P)],D,T) :- 
						infocus(A,P,Deg,T),
						emotion(A,fear,[P],D1,T0), T is T0+1,
						believe(A,not(P),D2,T), D is D1*D2*Deg.

cond_emotion(A,disappointment,[P],D,T) :-
						infocus(A,P,Deg,T),
						emotion(A,hope,[not(P)],D1,T0),
						T is T0+1,
						believe(A,P,D2,T),
						D is D1*D2*Deg.

% 2nd cas
cond_emotion(A,disappointment,[not(P)],D,T) :- 
						infocus(A,P,Deg,T),
						emotion(A,hope,[P],D1,T0), T is T0+1,
						believe(A,not(P),D2,T), D is D1*D2*Deg.

%%%%%%%%%%%%%%%%%%%%%%%%%
% more complex emotions %
%%%%%%%%%%%%%%%%%%%%%%%%%

% good-will
cond_emotion(A,happyfor(B),[P],D,T) :-
						infocus(A,P,Deg,T),believe(A,P,D1,T),
						believe(A,believe(B,P,_,Tfutur),D2,T),Tfutur >=T,
						% image of other, obtained via dialogue for instance
						believe(A,desire(B,P,D3),D4,T), 
						% desire deduced from bel(des()) via friendship rule (in demo.pl)
						desire(A,believe(B,P,_,_),D5),  
						D is D1*D2*D3*D4*D5*Deg.
						% most important degrees are D2,D3,D5

cond_emotion(A,sorryfor(B),[P],D,T) :-
						infocus(A,P,Deg,T),believe(A,P,D1,T),
						believe(A,believe(B,P,_,Tfutur),D2,T),Tfutur >=T,
						believe(A,undesire(B,P,D3),D4,T),
						undesire(A,believe(B,P,_,_),D5),
						D is D1*D2*D3*D4*D5*Deg.


% Ill will
cond_emotion(A,resentment(B),[P],D,T) :- infocus(A,P,Deg,T),believe(A,P,D1,T),
						   believe(A,believe(B,P,_,Tfutur),D2,T),Tfutur >=T,
						   believe(A,desire(B,P,D3),D4,T),
						   undesire(A,believe(B,P,_,_),D5),
						   D is D1*D2*D3*D4*D5*Deg.
cond_emotion(A,gloating(B),[P],D,T)   :- infocus(A,P,Deg,T),believe(A,P,D1,T),
						   believe(A,believe(B,P,_,Tfutur),D2,T),Tfutur >=T,
						   believe(A,undesire(B,P,D3),D4,T),
						   % desir deduit par la regle d'inimitie
						   desire(A,believe(B,P,_,_),D5), 
						   D is D1*D2*D3*D4*D5*Deg.

%%%%%%%%%%%%%%%%%%%%%%
% AGENT-BASED BRANCH %
%%%%%%%%%%%%%%%%%%%%%%

% self-agent action
cond_emotion(A,pride,[Alpha],D,T) :-
						infocus(A,Alpha,Degfocus,T),
						believe(A,action(A,Alpha,T1),Degbel1,T),
						T1 =< T, % l'instant de before alpha
						believe(A,unexpected(A,Alpha,DegDif),Degbel2,T1),
						believe(A,ideal(A,Alpha,DegIdl),Degbel3,T1),
						D is Degfocus*Degbel1*Degbel2*Degbel3*DegDif*DegIdl.

cond_emotion(A,shame,[Alpha],D,T) :-
						infocus(A,Alpha,Degfocus,T),
						believe(A,action(A,Alpha,T1),Degbel1,T),
						T1 =< T, % l'instant de before alpha
						believe(A,unexpected(A,Alpha,DegDif),Degbel2,T1),
						believe(A,unideal(A,Alpha,DegIdl),Degbel3,T1),
						D is Degfocus*Degbel1*Degbel2*Degbel3*DegDif*DegIdl.

% other agent action
cond_emotion(A,admiration(B),[Alpha],D,T) :-
						infocus(A,Alpha,Degfocus,T),
						believe(A,action(B,Alpha,T1),Degbel1,T),
						T1 =< T, % l'instant de before alpha
						believe(A,unexpected(B,Alpha,DegDif),Degbel2,T1),
						believe(A,ideal(B,Alpha,DegIdl),Degbel3,T1),
						D is Degfocus*Degbel1*Degbel2*Degbel3*DegDif*DegIdl.

cond_emotion(A,reproach(B),[Alpha],D,T) :-
						infocus(A,Alpha,Degfocus,T),
						believe(A,action(B,Alpha,T1),Degbel1,T),
						T1 =< T, % l'instant de before alpha
						believe(A,unexpected(B,Alpha,DegDif),Degbel2,T1),
						believe(A,unideal(B,Alpha,DegIdl),Degbel3,T1),
						D is Degfocus*Degbel1*Degbel2*Degbel3*DegDif*DegIdl.


%%%%%%%%%%%%%%%%%%%%%
% composed emotions %
%%%%%%%%%%%%%%%%%%%%%

% gratification = joy + pride
%% modification 17/11/2006 : added responsibility link between alpha and csq
%% TODO October 2012 : pleiad_resp file to handle responsibility

cond_emotion(A,gratification,[Alpha,Csq],D,T) :-
						%% joy existe (mais pas exprimee grace a f° d'expr°)
						cond_emotion(A,joy,[Csq],D1,T),
						%% pride existe (idem)
						cond_emotion(A,pride,[Alpha],D2,T),
						bel(A,resp(A,Alpha,Csq),D3,T,_),
						D is D1*D2*D3.

% remorse = sadness + shame
cond_emotion(A,remorse,[Alpha,Csq],D,T) :-
						cond_emotion(A,sadness,[Csq],D1,T),
						cond_emotion(A,shame,[Alpha],D2,T),
						bel(A,resp(A,Alpha,Csq),D3,T,_),
						D is D1*D2*D3.

% gratitude = joy + admiration
cond_emotion(A,gratitude(B),[Alpha,Csq],D,T) :-
						cond_emotion(A,joy,[Csq],D1,T),
						cond_emotion(A,admiration(B),[Alpha],D2,T),
						bel(A,resp(B,Alpha,Csq),D3,T,_),
						D is D1*D2*D3.

% anger = sadness + reproach
cond_emotion(A,anger(B),[Alpha,Csq],D,T) :-
						cond_emotion(A,sadness,[Csq],D1,T),
						cond_emotion(A,reproach(B),[Alpha],D2,T),
						bel(A,resp(B,Alpha,Csq),D3,T,_),
						D is D1*D2*D3.




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% conditions de maintien des emotions 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Rk October 2012 : totally unused conditions so commented
%cond_maintien(A,fear,[Prop],T) :- not(believe(A,Prop,_,T)),not(believe(A,not(Prop),_,T)),!.
%cond_maintien(A,hope,[Prop],T) :- not(believe(A,Prop,_,T)),not(believe(A,not(Prop),_,T)),!.
%cond_maintien(_,_,_,_) :- !.



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ONTOLOGY of emotions - OCC theory  %%
%%  positive and negative emotions    %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% defined differently in each emotiondef
% file (plug and play, one per theory)

positive(joy).
positive(hope).
positive(satisfaction).
positive(relief).
positive(happyfor(_)).
positive(gloating(_)).
positive(pride).
positive(admiration(_)).
positive(gratification).
positive(gratitude(_)).

negative(sadness).
negative(fear).
negative(fearconfirmed).
negative(disappointment).
negative(sorryfor(_)).
negative(resentment(_)).
negative(shame).
negative(reproach(_)).
negative(remorse).
negative(anger(_)).

emotiontype(E) :- positive(E).
emotiontype(E) :- negative(E).
emotiontype(empathy).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%   stressor extraction   %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% take a parameter E = emotion(A,NE,LP,D,T)
% NE = name of the emo
% LP = list of objects of the emo, Phi, or Alpha, or both
% not useful to extract stressor for positive emotions

%stresseur_phi(emotion(_,joy,[Phi],_,_),Phi).
stresseur_phi(emotion(_,sadness,[Phi],_,_),Phi).
%stresseur_phi(emotion(_,hope,[Phi],_,_),Phi).
stresseur_phi(emotion(_,fear,[Phi],_,_),Phi).
stresseur_phi(emotion(_,disappointment,[Phi],_,_),Phi).
stresseur_phi(emotion(_,fearconfirmed,[Phi],_,_),Phi).
stresseur_phi(emotion(_,sorryfor(_),[Phi],_,_),Phi).
stresseur_phi(emotion(_,resentment(_),[Phi],_,_),Phi).
stresseur_phi(emotion(_,anger(_),[_Alpha,Phi],_,_),Phi).
stresseur_phi(emotion(_,remorse,[_Alpha,Phi],_,_),Phi).

stresseur_alpha(emotion(_,shame,[Alpha],_,_),Alpha).
stresseur_alpha(emotion(_,reproach(_),[Alpha],_,_),Alpha).
stresseur_alpha(emotion(_,anger(_),[Alpha,_Phi],_,_),Alpha).
stresseur_alpha(emotion(_,remorse,[Alpha,_Phi],_,_),Alpha).

stresseur_agent(emotion(_,resentment(J),_,_,_),J).
stresseur_agent(emotion(_,reproach(J),_,_,_),J).
stresseur_agent(emotion(_,anger(J),_,_,_),J).
