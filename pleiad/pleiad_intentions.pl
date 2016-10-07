%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                                                       %%
%% 	PLEIAD Reasoning Engine			         %%
%%                                                       %%
%% Author : Jeremy Riviere, Carole Adam                  %%
%% Version : v1 - February 2012                          %%
%% Based on PLEIAD, developed by Carole Adam 2007        %%
%% Application to the CECIL project			 %%
%%							 %%
%%    ------- DO NOT DISTRIBUTE -------                  %%
%%                                                       %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% JUNE 2014 - from Inaffec/intention_deduction.pl
%% PROBLEM intend with 4 vs 5 parameters...
%% possible sources : nosource (in kbmgr/assert_intention, for retrocompat), coping (domi / emiliano), trust (yves)
%% global / localEmotion / localDialogue (Jeremy)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Intention Rule format %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%intention_rule(
	%preconds
%	(
	%% here the preconditions, coma separated
%	),
	%effects
	%% here the inference
%	).


% intention rules 777 - TODO add index to each rule
% to be used by pleiad_reasoning loop saturer_bc along with rules from various sources
rule(Index,T,Agt,Prem,Cci) :-  %current_agent(Agt),
    intention_rule(II,T,Agt,Prem,Cci),Index is 700+II.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Local Intention rules from emotions  %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% local emotion intention
% intention to express emotion if above expression threshold
intention_rule(1,Time,Agt,
		% preconds
		([
			%vecteur_emo_feel(Agt,Time,EmotionsFelt),
			max_emotion_felt(Agt,feel_emotion(Agt,E,P,DegEmo,Time)),
			%bel(Agt,before(Agt,SpeechAct,[feel_emotion(Agt,E,P,DegEmo,Time)|[_Constraints]]),_D,0,speech_rule),
			seuil_expression(Agt,DegSeuil),
			%not(believe(Agt,done(Agt,SpeechAct,_),_DegB,Time)),
			DegEmo >= DegSeuil
		]),
		% effects
		%intend(Agt,done(Agt,SpeechAct,_T1),1,Time,localEmotion)
		intend(Agt,bel(Agt,believe(_Hearer,feel_emotion(Agt,E,P,_D,Time),1,Time),1,Time,communication),1,Time,localEmotion) 
	).	
% FIXME pourquoi intention de croire que le hearer croit, plutot que directement intention que le hearer croie ?
    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Local Intention rules from the dialogue protocole  %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Adoption d'une intention suivant les regles de dialogues
intention_rule(2,T,Agt,
	% preconds
	([
	%T1 is T-1, % commented june2014 because now time is only moved after action
	believe(Agt,done(Speaker,SpeechActA,T),1,T),
	dialogue_rule(SpeechActA,SpeechActB,Precond),
	Precond,
	Speaker \= Agt
	]),
	% effects
	intend(Agt,done(Agt,SpeechActB,_),1,T,localDialogue)
	).
	
%% Keep the intention of redoing a speech act if this one is not successfull
			
intention_rule(3,T,Agt,
	% preconds
			([
                        %june2014 changed T-2 to T-1 for same reason as above
			T2 is T-1,
			believe(Agt,Prem -> after(Speaker,SpeechAct,_Effet),_Dbel,0),
			speaker(SpeechAct,Agt),
			ground(Speaker),				
			believe(Agt,Prem,_D,T),
			believe(Agt,done(Agt,SpeechAct,T2),_D1,T),
			not(successful(SpeechAct,T)),
			not(notsatisfied(SpeechAct,T))
			]),
	% effects
	%% Fevrier - quel T? l'acte a deja ete fait, donc il faut instacier le T; mais si il ya une emotion exprimee entre-temps, c'est T+1
			intend(Agt,done(Speaker,SpeechAct,T),0.8,T,localDialogue)
			).

	
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Global Intention rules  %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Adoption d'une intention quand on a le desir et qu'un plan existe pour l'atteindre

intention_rule(4,T,Agt,
	% preconds
			([
			%% My goal is to be responsible of Goal
			goal(Agt,resp(Agt,Goal,_T),Deg,T,_Src),
			%% I have committed myself (accepted a demand or promised)
			believe(Agt,committed(Agt,_,Goal),_D2,T),
			%% I don't have a global intention yet
			not(intend(Agt,_G,_D,T,global))
			]),
	% effects
			intend(Agt,Goal,Deg,T,global)
			).
			
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Inference Engine %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% New JUNE 2014 - agent as a parameter - saturate only this agent KB
% vecteur_intend is defined in pleiad_kbmgr.pl

% removed unused boucle_satur_intentions - intention_rules are declared as rules for the normal saturer_bc loop in pleiad_reasoning

