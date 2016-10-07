%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                                                       %%
%% PLEIAD : ProLog Emotional Intelligent Agents Designer %%
%%                                                       %%
%% Author : Carole Adam                                  %%
%% Version : v7 - July 2014                              %%
%%                                                       %%
%%    ------- DO NOT DISTRIBUTE -------                  %%
%%                                                       %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% summary of file
%% - 1) list of speech acts with components: hearer, speaker, illoc_force, prop_content
%% - 2) dialogue_rules (protocol)
%% - 3) success and satisfaction
%% - 4) consequences of actions and speech acts FIXME should be a normal demo rule...
%% - 5) has_more_to_say

%% library of speech acts is in pleiad_speechlib_jeremy.pl

%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 1) list of speech acts with components
%% from cecil / inaffec speech_act_component
%%%%%%%%%%%%%%%%%%%%%%%%%%
%% pleiad_speechacts_jeremy


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% conversion speech-acts -- actions %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% new for MOCA - June 2014
%% generic version
%% speechact(Force,Speaker, Hearer, CProp)
:- discontiguous(illoc_force/2,hearer/2,speaker/2,prop_content/2).
illoc_force(speechact(Force,_Speaker,_Hearer,_CProp),Force) :- !.
hearer(speechact(_Force,_Speaker,Hearer,_CProp),Hearer) :- !.
speaker(speechact(_Force,Speaker,_Hearer,_CProp),Speaker) :- !.
prop_content(speechact(_Force,_Speaker,_Hearer,CProp),CProp) :- !.





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2) dialogue rules (protocol)
%% from cecil_protocol (old version)
%% from dialogue_protocol (inaffec)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Dialogue rule format %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% dialogue_rule(speech act received, speech act to send) :- preconditions.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Dialogue rule List %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% I know a way to do it
% new june 2014 - new format for speechacts to allow quantifying on illoc force

%old format - dialogue_rule(ask(Speaker,Hearer,P),accept_demand(Hearer,Speaker,P),(find_one_plan(Hearer,P,_Action))).
%new format - dialogue_rule(SpeechactReceived,SpeechActResponse,Condition)

dialogue_rule(speechact(ask,Speaker,Hearer,P),speechact(accept_demand,Hearer,Speaker,P),(find_one_plan(Hearer,P,_Action))).

dialogue_rule(speechact(ask,Speaker,Hearer,P),speechact(refuse,Hearer,Speaker,P), ((not(find_one_plan(Hearer,P,_Action)));notgoal(Hearer,resp(Hearer,P,_T1),_D,_T,_S))).

dialogue_rule(speechact(demand,Speaker,Hearer,P),speechact(accept_demand,Hearer,Speaker,P),(instant(T), believe(Hearer,ami(Hearer,Speaker,1),_D,T))).

dialogue_rule(speechact(demand,Speaker,Hearer,P),speechact(refuse,Hearer,Speaker,P),((not(find_one_plan(Hearer,P,_Action)));notgoal(Hearer,resp(Hearer,P,_T1),_D,_T,_S))).

dialogue_rule(speechact(greet,Speaker,Hearer,saw(Speaker,Hearer)),speechact(remind,Hearer,Speaker,saw(Hearer,Speaker)),(believe(Hearer,believe(Speaker,saw(Hearer,Speaker),_D,_T),_D1,_T1))).

%dialogue_rule(speechact(ask_val,Speaker,Hearer,P),speechact(inform_val,Hearer,Speaker,value(P,Val)),(believe(Hearer,knowval(Hearer,P,Val),_D1,_T))).
dialogue_rule(speechact(ask_val,Speaker,Hearer,P),speechact(inform_val,Hearer,Speaker,value(P,Val)),(instant(T),believe(Hearer,value(P,Val),_D1,T))).

dialogue_rule(
                % received speech act
                speechact(ask_val,Speaker,Hearer,P),
                % protocol response speech act
                speechact(refuse,Hearer,Speaker,knowval(Speaker,P, T)),
                % condition
                ((instant(T),not(knowval(Hearer,P,T)));notgoal(Hearer,resp(Hearer,P,_T1),_D,_TG,_S))).

dialogue_rule(speechact(inform_val,Speaker,Hearer,value(P,Val)),speechact(remind,Hearer,Speaker,believe(Hearer,value(P,Val),Deg,T)),(instant(T), T1 is T-1, believe(Hearer,value(P,Val),Deg,T1))).

dialogue_rule(speechact(inform,Speaker,Hearer,P),speechact(remind,Hearer,Speaker,P),(instant(T), T1 is T-1, believe(Hearer,P,_D1,T1))).

dialogue_rule(speechact(inform,Speaker,Hearer,P),speechact(contradict,Hearer,Speaker,P),(instant(T), believe(Hearer,not(P),_D1,T))).

dialogue_rule(speechact(offer,Speaker,Hearer,P),speechact(accept_offer,Hearer,Speaker,P),(instant(T), goal(Hearer,resp(Speaker,P,T),_D4))).

dialogue_rule(speechact(offer,Speaker,Hearer,P),speechact(refuse,Hearer,Speaker,P),(instant(T), goal(Hearer,not(resp(Speaker,P,T)),_D4))).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3) SUCCESS CONDITIONS  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Success Condition   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% The Hearer has understood and answered the demand %%%%%%%%
%% The Speaker has to be the agent %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Check if the previous speech act performed by the agent is successful
previous_action_successful :- instant(T),
							current_agent(Agt),
							laststimulus(Agt,SpeechAct),
							speaker(SpeechAct,Agt),
							successful(SpeechAct,T),!.		
							
previous_action_successful :- instant(T),
							T2 is T-2,
							current_agent(Agt),
							laststimulus(Qqun,SpeechAct),
							speaker(SpeechAct,Qqun),
							Qqun \= Agt,
							%have to get the last speech act performed by the agent
							believe(Agt,done(Agt,SpeechAct2,T2),_D,T),
							successful(SpeechAct2,T),!.		
							
previous_action_successful :- current_agent(Agt),
							  laststimulus(Agt,action(_,_)).


% successful if it's the user who has spoken
successful(SpeechAct,T) :-  instant(T), speaker(SpeechAct,Speaker), ground(Speaker), not(current_agent(Speaker)).

% successful if success conditions are filled or satisfaction conditions are not satisfied
successful(SpeechAct,T) :- instant(T), speaker(SpeechAct,Speaker), ground(Speaker), believe(Speaker, done(Speaker,SpeechAct,_T),_D1,T),  (success(SpeechAct,T); notsatisfied(SpeechAct,T)).

success(speechact(ask_val,Speaker,Hearer,P),T) :- instant(T), believe(Speaker,done(Hearer,speechact(inform_val,Hearer,Speaker,value(P,_Val)),_),_D,T).

success(speechact(ask,Speaker,Hearer,P),T) :- instant(T),believe(Speaker,done(Hearer,speechact(accept_demand,Hearer,Speaker,P),_),_D,T).

success(speechact(demand,Speaker,Hearer,P),T) :- instant(T),believe(Speaker,done(Hearer,speechact(accept_demand,Hearer,Speaker,P),_),_D,T).

success(speechact(offer,Speaker,Hearer,P),T) :- instant(T),believe(Speaker,done(Hearer,speechact(accept_offer,Hearer,Speaker,P),_),_D,T).

success(SpeechAct,T) :- instant(T),
						illoc_force(SpeechAct,Force),
						ground(Force),
						Force \= ask_val, 
						Force \= ask, 
						Force \= demand, 
						Force \= offer. 
						
						
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% NoSatisfaction Condition   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% The Hearer has understood and answered negatively the demand %%%%%%%%
%% The Speaker has to be the agent %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% SpeechAct is not satisfied is the speaker is the agent and the hearer is refusing to answer
notsatisfied(SpeechAct,T) :- instant(T), speaker(SpeechAct,Speaker), ground(Speaker), believe(Speaker, done(Speaker,SpeechAct,_T),_D1,T), current_agent(Speaker), noSatisfaction(SpeechAct,T).

noSatisfaction(speechact(ask_val,Speaker,Hearer,P),T) :- instant(T), believe(Speaker,done(Hearer,speechact(refuse,Hearer,Speaker,knowval(Speaker, P, _Time)),_),_D,T).

% noSatisfaction(speechact(ask,Speaker,Hearer,P),T) :- instant(T), believe(Speaker,done(Hearer,speechact(refuse,Hearer,Speaker,speechact(accept_demand,Hearer,Speaker,P)),_),_D,T).

noSatisfaction(speechact(ask,Speaker,Hearer,P),T) :- instant(T), believe(Speaker,done(Hearer,speechact(refuse,Hearer,Speaker,P),_),_D,T).

noSatisfaction(speechact(demand,Speaker,Hearer,P),T) :- instant(T), believe(Speaker,done(Hearer,speechact(refuse,Hearer,Speaker,speechact(accept_demand,Hearer,Speaker,P)),_),_D,T).

noSatisfaction(speechact(offer,Speaker,Hearer,P),T) :- instant(T), believe(Speaker,done(Hearer,speechact(refuse,Hearer,Speaker,speechact(accept_offer,Hearer,Speaker,P)),_),_D,T).



%%%%%%%%%%%%%%%%%%%%%%
%% 4) CONSEQUENCES  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% rule for direct consequences of actions and speech acts %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% imported rule_csq predicate in Jeremy's inference_rule.pl file
%% to rules 221 to 225 in pleiad_demo (Nov2012 + July2014)
%% as a result predicate saturer_csq that used only these 2 rules should now be replaced with the standard saturer_bc


%%%%%%%%%%%%%%%%%%%%
%% 5) more to say %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Does the agent want to speak again?  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


has_more_to_say(Agt,T) :- instant(T),
						  laststimulus(Agt,action(_,_)),!. 
		
has_more_to_say(Agt,T) :- instant(T),
						  main_intention(Agt,Prop),
						  my_plans(Agt,Prop,[[action(greta,_)|_Plans]|_ListPlans]),!.
			
has_more_to_say(Agt,T) :- instant(T),
						  main_intention(Agt,Prop),
						  my_plans(Agt,Prop,[[SpeechAct|_Plans]|_ListPlans]),
						  speaker(SpeechAct,Agt),!.
 



