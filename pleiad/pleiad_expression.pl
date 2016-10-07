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


%% TODO October 2012
%% handle expression threshold

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% emotion expression module
%% will choose which emotion to express (instead of emo_intens)
%% e.g. based on agent personality (positive or negative)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% modifs 17/11/06 : express the last emotion instead of most intense one (to be more reactive)

% most recent emotion (warning: must be completely instantiated)
emo_recente(Agt,emotion(Agt,Emo,[Prop],Deg,Time),Time) :- laststimulus(Agt,event(_Evt,Prop)),
							emotion(Agt,Emo,[Prop],Deg,Time),
							not(emotion(Agt,Emo,[Prop],_D,Avant)),Avant is Time-1,!.

emo_recente(Agt,emotion(Agt,Emo,[Alpha],Deg,Time),Time) :- 
				laststimulus(Agt,action(_Resp,Alpha,_Csq)),
				emotion(Agt,Emo,[Alpha],Deg,Time),
				not((emotion(Agt,Emo,[Alpha],_D,Avant),Avant is Time-1)),!.

emo_recente(Agt,emotion(Agt,Emo,[Alpha,Csq],Deg,Time),Time) :- 
					laststimulus(Agt,action(_Resp,Alpha,Csq)),
					emotion(Agt,Emo,[Alpha,Csq],Deg,Time),
					not((emotion(Agt,Emo,[Alpha,Csq],_D,Avant),Avant is Time-1)),!.


% otherwise default base case: still express an emotion = the most intense one
% on condition that emo_recente is neutral
emo_intens(Agt,E,T) :- emo_recente(Agt,E,T).  % should take the most intense of all..
emo_intens(Agt,E,T) :- not(emo_recente(_A,_E,_T)),vecteur_emo(Agt,T,V),sort_emo(V,[E|_]).

express_emotion(A,emotion(A,joy,[P],D,T),T) :- 
								emo_intens(A,emotion(A,joy,[P],D,T),T),
								% priority to confirm-based emotions
								not(emotion(A,satisfaction,[P],_,T)),
								not(emotion(A,relief,[P],_,T)),
								% priority to emotions towards the responsible
								% (depending on the agent's personality)
								not((believe(A,resp(B,Alpha,P),_,T),emotion(A,gratitude(B),[Alpha,P],_,T))),
								not((believe(A,resp(A,Alpha,P),_,T),emotion(A,gratification,[Alpha,P],_,T))),!.
								  
express_emotion(A,emotion(A,sadness,[P],D,T),T) :- 
								emo_intens(A,emotion(A,sadness,[P],D,T),T),
								% priority to confirmation-based emotions
								not(emotion(A,fearconfirmed,[P],_,T)),
								not(emotion(A,disappointment,[P],_,T)),
								% priority to emotions towards the responsible agents
								% (if the agent is of rancorous personality)
								not((believe(A,resp(B,Alpha,P),_,T),emotion(A,anger(B),[Alpha,P],_,T))),
								not((believe(A,resp(A,Alpha,P),_,T),emotion(A,remorse,[Alpha,P],_,T))),!.

%% expression of confirmation-based emotions
express_emotion(A,emotion(A,satisfaction,[P],D,T),T) :- 
			emo_intens(A,emotion(A,joy,[P],_,T),T),
			emotion(A,satisfaction,[P],D,T),!.
								
express_emotion(A,emotion(A,fearconfirmed,[P],D,T),T) :- 
			emo_intens(A,emotion(A,sadness,[P],_,T),T),
			emotion(A,fearconfirmed,[P],D,T),!.
																
%% and of disconfirm-based emotions
express_emotion(A,emotion(A,disappointment,[P],D,T),T) :- 
			emo_intens(A,emotion(A,sadness,[P],_,T),T),
			emotion(A,disappointment,[P],D,T),!.
								
express_emotion(A,emotion(A,relief,[P],D,T),T) :- 
			emo_intens(A,emotion(A,joy,[P],_,T),T),
			emotion(A,relief,[P],D,T),!.

% the agent only expresses pride, shame, reproach, admiration
% if they are not part of a compound emotion that is also triggered

express_emotion(A,emotion(A,pride,[Alpha],D,T),T) :- 
		emo_intens(A,emotion(A,pride,[Alpha],D,T),T),
		not(emotion(A,gratification,[Alpha,_],_,T)),!.

express_emotion(A,emotion(A,shame,[Alpha],D,T),T) :- 
		emo_intens(A,emotion(A,shame,[Alpha],D,T),T),
		not(emotion(A,remorse,[Alpha,_],_,T)),!.
											
express_emotion(A,emotion(A,reproach(B),[Alpha],D,T),T) :- 
		emo_intens(A,emotion(A,reproach(B),[Alpha],D,T),T),
		not(emotion(A,anger(B),[Alpha,_],_,T)),!.

express_emotion(A,emotion(A,admiration(B),[Alpha],D,T),T) :- 
		emo_intens(A,emotion(A,admiration(B),[Alpha],D,T),T),
		not(emotion(A,gratitude(B),[Alpha,_],_,T)),!.


%% complex emotions are expressed even if they are not the most intense (cf emo confirm)
%% on condition that one of the two component emotions (wellbeing or attribution) is the most intense

express_emotion(A,emotion(A,gratification,[Alpha,P],D,T),T) :- 
		emo_intens(A,emotion(A,joy,[P],_,T),T),
		emotion(A,gratification,[Alpha,P],D,T),!.

express_emotion(A,emotion(A,gratification,[Alpha,P],D,T),T) :- 
		emo_intens(A,emotion(A,pride,[Alpha],_,T),T),
		emotion(A,gratification,[Alpha,P],D,T),!.

express_emotion(A,emotion(A,remorse,[Alpha,P],D,T),T) :- 
		emo_intens(A,emotion(A,sadness,[P],_,T),T),
		emotion(A,remorse,[Alpha,P],D,T),!.

express_emotion(A,emotion(A,remorse,[Alpha,P],D,T),T) :- 
		emo_intens(A,emotion(A,shame,[Alpha],_,T),T),
		emotion(A,remorse,[Alpha,P],D,T),!.

express_emotion(A,emotion(A,gratitude(B),[Alpha,P],D,T),T) :- 
		emo_intens(A,emotion(A,joy,[P],_,T),T),
		emotion(A,gratitude(B),[Alpha,P],D,T),!.

express_emotion(A,emotion(A,gretitude(B),[Alpha,P],D,T),T) :- 
		emo_intens(A,emotion(A,admiration(B),[Alpha],_,T),T),
		emotion(A,gratitude(B),[Alpha,P],D,T),!.

express_emotion(A,emotion(A,anger(B),[Alpha,P],D,T),T) :- 
		emo_intens(A,emotion(A,sadness,[P],_,T),T),
		emotion(A,anger(B),[Alpha,P],D,T),!.

express_emotion(A,emotion(A,anger(B),[Alpha,P],D,T),T) :- 
		emo_intens(A,emotion(A,reproach(B),[Alpha],_,T),T),
		emotion(A,anger(B),[Alpha,P],D,T),!.


%% for all emotion that have no specific condition 
%% (warning : if specific conditions were forgotten, this rule will trigger the wrong emotion)
express_emotion(A,E,T) :- emo_intens(A,E,T),!.
