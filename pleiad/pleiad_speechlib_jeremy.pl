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

% language for Jeremy's MCL
% - 5) library of speech acts with conditions (before, after)
% new june 2014 - reformule la librairie (composants, conditions)

% debugtag = lang

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 0) discontiguous predicates  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

:- discontiguous(send_condition/2).
:- discontiguous(send_effects/2).
:- discontiguous(receive_effects/2).



%%%%%%%%%%%%%%%%%%
%% 5) LIBRARY   %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 5) library of speech acts with before and after  %%
%% from cecil_speech in pleiad-v8 (old version)     %%
%% from speech_act.pl in inaffec                    %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% syntax :
%   - speech act as action : speechact(Force,Speaker,Hearer,Cprop)
%   - send_condition(SpeechAct,[Cond]) ou send_condition(Speechact,[Cond,Contraintes])
%   - send_effects(SpeechAct,Effect)
%   - receive_effects(SpeechAct,Effect)

						  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% SPEECH ACT'S PRECONDITION AND EFFECT %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% preconditions: we make the difference between the preparatory conditions (they have to be true in the context)
% and the illocutionary goal (the agent has the goal to have the act's effects)
% because we cannot check that the speaker has the goal to perform this action in its preconditions
% because if we do, then the speaker doesnt adopt the goal to perform this action because it is unfeasible as long as he doesn't have this goal... cycle!!!						  

% new syntax june2014 - no negative consequence ?
% speechact( Agent,Time,force,speaker,hearer, propcontent,before,constraint,interpretcsq,receivecsq)
% il faudra faire

assert_language :-
			instant(T),current_agent(Agt),
                    % assert all preconditions as before - speaker is the agent
                    forall(
                        ( 
				send_condition(SA,C),
				speaker(SA,Agt),
				human(H),
				hearer(SA,H),
				%Agt \= Human,
				debug_message(lang,['send condition ',C,' of speech act ',SA,' converted to before'])
			),
                        kassert(bel(Agt,before(Agt,SA,C),1,T,speech_rule))
                    ),
                    % assert all send_effect as after - speaker is the agent
                    forall(
                        (
				send_effects(SA,E),
				speaker(SA,Agt),
				human(H),
				hearer(SA,H),
				%Agt \= Human,
				debug_message(lang,['send effect ',E,' of speech act ',SA,' converted to after'])
			),
                        kassert(bel(Agt,after(Agt,SA,E),1,T,speech_rule))
                    ),
                    % assert all receive effect as after - hearer is the agent
                    forall(
                        (
				receive_effects(SA,E),
				hearer(SA,Agt),
				human(Speaker),
				speaker(SA,Speaker),
				%Agt \= Speaker,
				debug_message(lang,['receive effect ',E,' of speech act ',SA,' converted to after'])
			),
                        kassert(bel(Agt,after(Speaker,SA,E),1,T,speech_rule))
                    ).


%% NEW JUNE 2015
%% need to specify who the two interlocutors are. One is current_agent, the other is the human in the scenario.
%% use human(X) predicate to constrain possible values of interlocutor of agent in scenario




                    
% to be used to check if an INSTANTIATED speech act is sendable
% if Speechact not grounded : returns true but does not instantiate Speechact 
% FIXME - added check grounded speech act - is it necessary? will it stop for sending speechacts where one param is not instantiated?
sendable(Speechact) :- findall(C,send_condition(Speechact,C),LC), forall(member(C,LC),valid(C)),ground(Speechact).

% TODO later : for robots, could add one more condition for any speech act to be sendable: be facing the hearer
% (ie being in the same room, looking at him) -> this will add physical actions to the plan, for the robot to move there

% check if a speech act establishes a proposition
%establish(ActAgt,Speechact,Prop) :- (send_effects(Speechact,Prop);receive_effects(Speechact,Prop)),speaker(Speechact,ActAgt).%% 

% FIXME : should be merged with establishing_action for physical actions, ie using only before/after as asserted by assert_language
% this predicate establish should be REMOVED - TODO july2014
% assert_language should ensure the conditions checked here

% case 1 - establishes as a receive effect -> planning agent is the hearer
establish(PlanAgt,Speechact,GProp) :- receive_effects(Speechact,BProp),achieves_goal(GProp,BProp),hearer(Speechact,PlanAgt),speakerNOThearer(Speechact).
% case 2 - establishes as a send effect -> planning agent is the speaker
establish(PlanAgt,Speechact,GProp) :- send_effects(Speechact,BProp),achieves_goal(GProp,BProp),speaker(Speechact,PlanAgt),speakerNOThearer(Speechact).

% Add condition that speaker should be different from hearer : hearer(Speechact,Hearer),Hearer\=ActAgt.
% if both instantiated, should be different
speakerNOThearer(SA) :- speaker(SA,Spk),ground(Spk),hearer(SA,Hr),ground(Hr),Spk\=Hr.
% if one not instantiated: ok
speakerNOThearer(SA) :- speaker(SA,Spk),hearer(SA,Hr),(not(ground(Hr));not(ground(Spk))).


%%%%%%%%%%%%%%
%%%%%%%%%%%%
%%%%%%%%%%
%%%%%%%%
%%%%%%
%%%%
%%
% preconditions and effects of MCA - cf table in Annexe p.150 of Jeremy PhD

%%%%%%%%%%%%%%%%%%%
%-> Assertive MCA %
%%%%%%%%%%%%%%%%%

%%%%%%%%%
%-> Inform
%%%%%%%%%

%%%%%%
%% PRECONDITIONS
%%%%%%
% send_conditions - both for the agent and the speaker (preconditions used in planning)
send_condition(speechact(inform,Speaker,_Hearer,P),[believe(Speaker,P,Db,T),Db>0.3]) :- instant(T).
send_condition(speechact(inform,Speaker,Hearer,P),[not(believe(Speaker,believe(Hearer,P,_D1,_T1),_D2,_T2))]).

%%%%%%
%% EFFECTS
%%%%%%

% interpret conseq
send_effects(speechact(inform,_Speaker,Hearer,P),believe(Hearer,P,0.75,T)) :- instant(T).

% receive conseq
%bel(Agt,instant(T)->after(Speaker,inform(Speaker,Agt,P),bel(Agt,goal(Speaker,believe(Agt,P,1,T),0.7,T,_Src),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
receive_effects(speechact(inform,Speaker,Hearer,P),goal(Speaker,believe(Hearer,P,1,T),0.7,T,_Src)) :- instant(T).

%bel(Agt,instant(T)->after(Speaker,inform(Speaker,Agt,P),bel(Agt,believe(Speaker,P,0.75,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
receive_effects(speechact(inform,Speaker,_Hearer,P),believe(Speaker,P,0.75,T)) :- instant(T).



%%%%%%%%%
%-> Inform_Val = specific case of inform
%%%%%%%%%

%%%%%%
%% PRECONDITIONS
%%%%%%

% send_conditions
%bel(Agt,before(Speaker,inform_val(Speaker,Hearer,value(P,Val)),[bel(Agt,believe(Speaker,knowval(Speaker,P,Val),Db,_T),_Dbp,_Tbp),[ground(Hearer),Db>0.7,atom(P)]]),1,0,speech_rule) :- current_agent(Agt).
send_condition(speechact(inform_val,Speaker,_Hearer,value(P,Val)),[believe(Speaker,value(P,Val),Db,T),Db>0.7,atom(P)]) :- instant(T).

%bel(Agt,before(Speaker,inform_val(Speaker,Hearer,value(P,Val)),[bel(Agt,believe(Speaker,goal(Hearer,knowval(Hearer,P,Val),_Dg),_D,_T,_Src),_Dbp,_Tbp),[ground(Hearer),atom(P)]]),1,0,speech_rule) :- current_agent(Agt).
send_condition(speechact(inform_val,Speaker,Hearer,value(P,_Val)),[believe(Speaker,goal(Hearer,knowval(Hearer,P,_Sometime),_Dg),_Db,T),atom(P)]) :- instant(T).

%bel(Agt,before(Speaker,inform_val(Speaker,Hearer,value(P,Val)),[not(bel(Agt,believe(Speaker,knowval(Hearer,P,Val),_D2,_T1),_Dbp,_Tbp)),[ground(Hearer),atom(P)]]),1,0,speech_rule) :- current_agent(Agt).
% slight change: instead of do not believe hearer knowval of P, it is now do not believe hearer believes the SAME val of P (so can send new contradictory value)
send_condition(speechact(inform_val,Speaker,Hearer,value(P,Val)),[not(believe(Speaker,believe(Hearer,value(P,Val),_Dh,_Th),_Db,T)),ground(Hearer),atom(P)]) :- instant(T).

%%%%%%
%% EFFECTS
%%%%%%

% interpret conseq
%bel(Agt,instant(T)->after(Agt,inform_val(Agt,Hearer,value(P,Val)),bel(Agt,knowval(Hearer,P,Val),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
% after sending speech act, the speaker believes that the hearer trusted him totally on the information...
% FIXME trust : should be bel(spk,bel(hr,bel(spk,value(p,val)))) only, but makes planning harder because this is not spk goal
% FIXME trust : at least degree of adopted belief should depend on trust between spk and hr
send_effects(speechact(inform_val,Speaker,Hearer,value(P,Val)),believe(Speaker,believe(Hearer,value(P,Val),_Deg,T),1,T)) :- instant(T).

% receive conseq
%bel(Agt,instant(_T)->after(Speaker,inform_val(Speaker,Agt,value(P,Val)),knowval(Agt,P,Val)),1,0,speech_rule) :- current_agent(Agt).
% FIXME trust: degree of adopted belief should depend on trust between spk and hr
receive_effects(speechact(inform_val,_Speaker,Hearer,value(P,Val)),believe(Hearer,value(P,Val),1,T)) :- instant(T).

%bel(Agt,instant(T)->after(Speaker,inform_val(Speaker,Agt,value(P,Val)),bel(Agt,knowval(Speaker,P,Val),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
% FIXME degrees should not always be 1 but depend on trust, intensity of inform-val (could be one more parameter of speech act)
receive_effects(speechact(inform_val,Speaker,Hearer,value(P,Val)),believe(Hearer,believe(Speaker,value(P,Val),1,T),1,T)) :- instant(T).

%bel(Agt,instant(T)->after(Speaker,inform_val(Speaker,Agt,value(P,Val)),bel(Agt,believe(Speaker,knowval(Agt,P,Val),0.75,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
receive_effects(speechact(inform_val,Speaker,Hearer,value(P,Val)),believe(Speaker,knowval(Hearer,P,Val),1,T)) :- instant(T).


%%%%%%%%%
%-> Say
%%%%%%%%%



%%%%%%%%%
%-> Affirm
%%%%%%%%%

%%%%%%%%%
%-> Deny
%%%%%%%%%

%%%%%%%%%
%-> Guess
%%%%%%%%%


%%%%%%%%%
%-> Contradict
%%%%%%%%%

%%%%%%%%%
%-> Remind
%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%
%-> Commissive MCA %
%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%
%-> Promise
%%%%%%%%%


% send_conditions
%old%bel(Agt,before(Speaker,promise(Speaker,Hearer,P),[ground(Hearer), [bel(Agt,goal(Speaker,believe(Hearer,goal(Speaker,resp(Speaker,P,_T),_D1),_D2,_T1),D3),_Dbp,_Tbp), D3<0.75]]),1,0,speech_rule) :- current_agent(Agt).
%old%bel(Agt,before(Speaker,promise(Speaker,Hearer,P),[ground(Hearer), [bel(Agt,goal(Speaker,resp(Speaker,P,_T),_D1),_Dbp,_Tbp)]]),1,0,speech_rule) :- current_agent(Agt).
%new%bel(Agt,before(Speaker,promise(Speaker,Hearer,P),[bel(Agt,believe(Speaker,goal(Hearer,P,_D5),_D6,_T),_Dbp,_Tbp),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).
send_condition(speechact(promise,Speaker,Hearer,P),[believe(Speaker,goal(Hearer,P,_D5),_Db,T)]) :- instant(T).

% interpret conseq
%bel(Agt,instant(T)->after(Agt,promise(Agt,Hearer,P),bel(Agt,committed(Agt,Hearer,P),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
send_effects(speechact(promise,Speaker,Hearer,P),committed(Speaker,Hearer,P)). % :- instant(T).

%bel(Agt,instant(T)->after(Agt,promise(Agt,Hearer,P),bel(Agt,believe(Hearer, goal(Agt,resp(Agt,P,T),0.7),1,T),1.0,T,communication)),1,0,speech_rule) :- current_agent(Agt).
send_effects(speechact(promise,Speaker,Hearer,P),believe(Speaker,believe(Hearer,goal(Speaker,resp(Speaker,P,T),0.7),1,T),1,T)) :- instant(T).

% receive conseq
bel(Agt,instant(T)->after(Speaker,promise(Speaker,Agt,P),bel(Agt,goal(Speaker,resp(Speaker,P,T),0.7),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,promise(Speaker,Agt,P),bel(Agt,believe(Speaker,goal(Agt,P,0.7),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).


%%%%%%%%%
%-> Assure
%%%%%%%%%

%%%%%%%%
%-> Accept_demand
% accept an action performed by self
%%%%%%%%%

%%%%%%%%%
%-> Accept_offer
% accept an action performed by the Hearer
%%%%%%%%%

%%%%%%%%%
%-> Refuse
%%%%%%%%%


%%%%%%%%%
%-> Offer
%%%%%%%%%


%%%%%%%%%
%-> Renounce
%%%%%%%%%







%%%%%%%%%%%%%%%%%%%%
%-> Directive MCA %%
%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%
%-> Ask
%%%%%%%%%

%%%%%%%%%

%%%%%%%%%
%-> Ask_val
%%%%%%%%%


%%%%%%%%%
%-> Demand
%%%%%%%%%

%%%%%%%%%
%-> Advise
%%%%%%%%%

%%%%%%%%%
%-> Suggest
%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%
%-> Expressive MCA %
%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%
%-> Greet %%
%%%%%%%%%%%%

%%%%%%%%%%%%
%-> Cry %%
%%%%%%%%%%%%

%%%%%%%%%%%%
%-> Approve %%
%%%%%%%%%%%%


%%%%%%%%%%%%
%-> Disapprove %%
%%%%%%%%%%%%



%%%%%%%%%%%%%%
%-> Rejoice %%
%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%
%-> Be Very Pleased %%
%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%
%-> Thank %%
%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%
%-> Congratulate %%
%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%
%-> Regret %%
%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%
%-> Apologize %%
%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%
%-> Beg Pardon %%
%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%
%-> Complain %%
%%%%%%%%%%%%%%%


%%%%%%%%%%%
%-> Moan %%
%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%
%-> Feel Guilty %%
%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%
%-> Suffer For %%
%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%
%-> Reproach %%%%%
%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%
%-> Protest %%%%%
%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%
%-> Satisfy %%
%%%%%%%%%%%%%%

%%%%%%%%%%%%
%-> Boast %%
%%%%%%%%%%%%


%%%%%%%%%%%%%%%%
%-> Subscribe %%
%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%
%-> Compliment %%
%%%%%%%%%%%%%%%%%

