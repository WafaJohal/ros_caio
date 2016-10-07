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
% note June2014 - incomplete - cf cecil_speech+conv+protocol in pleiad-v8-dialogue

%% summary of file
%% - 1) list of speech acts with components: hearer, speaker, illoc_force, prop_content
%% - 2) dialogue_rules (protocol)
%% - 3) success and satisfaction
%% - 4) has_more_to_say
%% - 5) library of speech acts with conditions (before, after)

% new june 2014 - reformule la librairie (composants, conditions)


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
% speechact(Agent,Time,force,speaker,hearer,propcontent,before,constraint,interpretcsq,receivecsq)
% il faudra faire

assert_language :- current_agent(Agt), instant(T),
                    % assert all preconditions
                    forall(
                           (send_condition(SA,C),speaker(SA,Speaker)),
                            kassert(bel(Agt,before(Speaker,SA,C),1,T,speech_rule))
                    ),
                    % assert all send_effect
                    forall(
                            (send_effects(SA,E),speaker(SA,Speaker)),
                             kassert(bel(Agt,after(Speaker,SA,E),1,T,speech_rule))
                    ),
                    % assert all receive effect
                    forall(
                            (receive_effects(SA,E),speaker(SA,Speaker)),
                            kassert(bel(Agt,after(Speaker,SA,E),1,T,speech_rule))
                    ).
                    
% to be used to check if an INSTANTIATED speech act is sendable
% if Speechact not grounded : returns true but does not instantiate Speechact 
% FIXME - added check grounded speech act - is it necessary? will it stop for sending speechacts where one param is not instantiated?
sendable(Speechact) :- findall(C,send_condition(Speechact,C),LC), forall(member(C,LC),valid(C)),ground(Speechact).

% TODO later : for robots, could add one more condition for any speech act to be sendable: be facing the hearer
% (ie being in the same room, looking at him) -> this will add physical actions to the plan, for the robot to move there

% check if a speech act establishes a proposition
%establish(ActAgt,Speechact,Prop) :- (send_effects(Speechact,Prop);receive_effects(Speechact,Prop)),speaker(Speechact,ActAgt).

% case 1 - establishes as a receive effect -> planning agent is the hearer
establish(PlanAgt,Speechact,GProp) :- receive_effects(Speechact,BProp),achieves_goal(GProp,BProp),hearer(Speechact,PlanAgt).
% case 2 - establishes as a send effect -> planning agent is the speaker
establish(PlanAgt,Speechact,GProp) :- send_effects(Speechact,BProp),achieves_goal(GProp,BProp),speaker(Speechact,PlanAgt).

% FIXME pb in establish: if prop is obtained with a higher degree than intended, it should work too
% (for now only unifies if exact same degree)

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
send_condition(speechact(inform,Speaker,_Hearer,P),[bel(Speaker,P,Db,T,_S),Db>0.3]) :- instant(T).
send_condition(speechact(inform,Speaker,Hearer,P),[not(bel(Speaker,believe(Hearer,P,_D1,_T1),_D2,_T2,_S2))]).

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
send_condition(speechact(inform_val,Speaker,_Hearer,value(P,Val)),[believe(Speaker,knowval(Speaker,P,Val),Db,T),Db>0.7,atom(P)]) :- instant(T).

%bel(Agt,before(Speaker,inform_val(Speaker,Hearer,value(P,Val)),[bel(Agt,believe(Speaker,goal(Hearer,knowval(Hearer,P,Val),_Dg),_D,_T,_Src),_Dbp,_Tbp),[ground(Hearer),atom(P)]]),1,0,speech_rule) :- current_agent(Agt).
send_condition(speechact(inform_val,Speaker,Hearer,value(P,Val)),[believe(Speaker,goal(Hearer,knowval(Hearer,P,Val),_Dg),_Db,T),atom(P)]) :- instant(T).

%bel(Agt,before(Speaker,inform_val(Speaker,Hearer,value(P,Val)),[not(bel(Agt,believe(Speaker,knowval(Hearer,P,Val),_D2,_T1),_Dbp,_Tbp)),[ground(Hearer),atom(P)]]),1,0,speech_rule) :- current_agent(Agt).
send_condition(speechact(inform_val,Speaker,Hearer,value(P,Val)),[not(believe(Speaker,knowval(Hearer,P,Val),_Db,T)),ground(Hearer),atom(P)]) :- instant(T).

%%%%%%
%% EFFECTS
%%%%%%

% interpret conseq
%bel(Agt,instant(T)->after(Agt,inform_val(Agt,Hearer,value(P,Val)),bel(Agt,knowval(Hearer,P,Val),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
send_effects(speechact(inform_val,Speaker,Hearer,value(P,Val)),believe(Speaker,knowval(Hearer,P,Val),1,T)) :- instant(T).


% receive conseq
%bel(Agt,instant(_T)->after(Speaker,inform_val(Speaker,Agt,value(P,Val)),knowval(Agt,P,Val)),1,0,speech_rule) :- current_agent(Agt).
receive_effects(speechact(inform_val,_Speaker,Hearer,value(P,Val)),believe(Hearer,knowval(Hearer,P,Val),1,T)) :- instant(T).

%bel(Agt,instant(T)->after(Speaker,inform_val(Speaker,Agt,value(P,Val)),bel(Agt,knowval(Speaker,P,Val),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
receive_effects(speechact(inform_val,Speaker,Hearer,value(P,Val)),believe(Hearer,knowval(Speaker,P,Val),1,T)) :- instant(T).

%bel(Agt,instant(T)->after(Speaker,inform_val(Speaker,Agt,value(P,Val)),bel(Agt,believe(Speaker,knowval(Agt,P,Val),0.75,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
receive_effects(speechact(inform_val,Speaker,Hearer,value(P,Val)),believe(Speaker,knowval(Hearer,P,Val),1,T)) :- instant(T).

%--

%%%%%%%%%
%-> Say
%%%%%%%%%

% %send_conditions
bel(Agt,before(Speaker,say(Speaker,Hearer,P),[bel(Agt,believe(Speaker,P,_Db,_T),_Dbp,_Tbp),[ground(Hearer),atom(P)]]),1,0,speech_rule) :- current_agent(Agt).
%bel(Agt,before(Speaker,say(Speaker,Hearer,P),[ground(Hearer), [bel(Agt,goal(Speaker,believe(Hearer,P,_D,_T),Dg,_Tg,_Srcg),_Dbp,_Tbp),Dg>=0.75,atom(P)]]),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,before(Speaker,say(Speaker,Hearer,P),[not(bel(Agt,believe(Speaker,believe(Hearer,P,_D1,_T),_D2,_T1),_Dbp,_Tbp)),[ground(Hearer),atom(P)]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,say(Agt,Hearer,P),bel(Agt,believe(Hearer,P,0.8,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,say(Speaker,Agt,P),bel(Agt,goal(Speaker,believe(Agt,P,1,T),0.8,T,_Src),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,say(Speaker,Agt,P),bel(Agt,believe(Speaker,P,0.8,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

%--
%%%%%%%%%
%-> Affirm
%%%%%%%%%

% %send_conditions
bel(Agt,before(Speaker,affirm(Speaker,Hearer,P),[bel(Agt,believe(Speaker,P,_Db,_T),_Dbp,_Tbp),[ground(Hearer),atom(P)]]),1,0,speech_rule) :- current_agent(Agt).
%bel(Agt,before(Speaker,affirm(Speaker,Hearer,P),[ground(Hearer), [bel(Agt,goal(Speaker,believe(Hearer,P,_D,_T),1,T,_Src),_Dbp,_Tbp),atom(P)]]),1,0,speech_rule) :- current_agent(Agt).

% % interpret conseq
bel(Agt,instant(T)->after(Agt,affirm(Agt,Hearer,P),bel(Agt,believe(Hearer,P,1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% % receive conseq
bel(Agt,instant(T)->after(Speaker,affirm(Speaker,Agt,P),bel(Agt,goal(Speaker,believe(Agt,P,1,T),0.6,T,_Src),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,affirm(Speaker,Agt,P),bel(Agt,believe(Speaker,P,1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

%--
%%%%%%%%%
%-> Deny
%%%%%%%%%

% send_conditions
bel(Agt,before(Speaker,deny(Speaker,Hearer,P),[bel(Agt,believe(Speaker,not(P),_Db,_T),_Dbp,_Tbp), [ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).
%bel(Agt,before(Speaker,deny(Speaker,Hearer,P),[ground(Hearer), [bel(Agt,goal(Speaker,believe(Hearer,not(P),1,_T),_Dg),_Dbp,_Tbp)]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,deny(Agt,Hearer,P),bel(Agt,believe(Hearer,not(P),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,deny(Speaker,Agt,P),bel(Agt,goal(Speaker,believe(Agt,not(P),1,T),0.6),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,deny(Speaker,Agt,P),bel(Agt,believe(Speaker,not(P),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
%--

%%%%%%%%%
%-> Guess
%%%%%%%%%

% send_conditions
bel(Agt,before(Speaker,guess(Speaker,Hearer,P),[bel(Agt,prob(Speaker,P,Db,_T),_Dbp,_Tbp),[ground(Hearer),Db=<0.3,atom(P)]]),1,0,speech_rule) :- current_agent(Agt).
%bel(Agt,before(Speaker,guess(Speaker,Hearer,P),[ground(Hearer), [bel(Agt,goal(Speaker,prob(Hearer,P,Db,_T),_Dg),_Dbp,_Tbp),Db=<0.5,atom(P)]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,guess(Agt,Hearer,P),bel(Agt,prob(Hearer,P,0.5,T),0.5,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,guess(Speaker,Agt,P),bel(Agt,goal(Speaker,prob(Agt,P,0.5,T),0.6),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,guess(Speaker,Agt,P),bel(Agt,prob(Speaker,P,0.5,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
%--
%%%%%%%%%
%-> Contradict
%%%%%%%%%

% send_conditions
bel(Agt,before(Speaker,contradict(Speaker,Hearer,not(P)),[bel(Agt,believe(Speaker,P,_Db,_T),_Dbp,_Tbp),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).
%bel(Agt,before(Speaker,contradict(Speaker,Hearer,not(P)),[ground(Hearer), [bel(Agt,goal(Speaker,believe(Hearer,P,_Db,_T),_Dg),_Dbp,_Tbp)]]),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,before(Speaker,contradict(Speaker,Hearer,not(P)),[bel(Agt,believe(Hearer,not(P),_D,_T),_Dbp,_Tbp), [ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).

bel(Agt,before(Speaker,contradict(Speaker,Hearer,P),[ground(Hearer), [bel(Agt,believe(Speaker,not(P),_Db,_T),_Dbp,_Tbp)]]),1,0,speech_rule) :- current_agent(Agt), atom(P).
%bel(Agt,before(Speaker,contradict(Speaker,Hearer,P),[ground(Hearer), [bel(Agt,goal(Speaker,believe(Hearer,not(P),_Db,_T),_Dg),_Dbp,_Tbp),atom(P)]]),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,before(Speaker,contradict(Speaker,Hearer,P),[ground(Hearer), [bel(Agt,believe(Hearer,P,_D,_T),_Dbp,_Tbp)]]),1,0,speech_rule) :- current_agent(Agt), atom(P).

% interpret conseq
bel(Agt,instant(T)->after(Agt,contradict(Agt,Hearer,P),bel(Agt,believe(Hearer,believe(Agt,P,1,T),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,contradict(Speaker,Agt,P),bel(Agt,goal(Speaker,believe(Agt,P,1,T),0.6),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,contradict(Speaker,Agt,P),bel(Agt,believe(Speaker,P,1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

%--
%%%%%%%%%
%-> Remind
%%%%%%%%%

% send_conditions
bel(Agt,before(Speaker,remind(Speaker,Hearer,P),[bel(Agt,believe(Speaker,P,_Db,_T),_Dbp,_Tbp),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).
% commenté: si le remind est déclenché en réponse d'un acte, ceci est trop restrictif (A VOIR) - remplacé par la dernière ligne
% (bel(Agt,before(Agt,remind(Agt,Hearer,P),[ground(Hearer), [goal(Agt,believe(Hearer,P,D1,T),_Dg),believe(Agt,believe(Hearer,P,D2,T),_D,T),D2<D1]]),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,before(Speaker,remind(Speaker,Hearer,P),[bel(Agt,believe(Speaker,believe(Hearer,P,_D2,_T),_D,_T1),_Dbp,_Tbp),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,remind(Agt,Hearer,P),bel(Agt,believe(Hearer,P,0.75,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,remind(Speaker,Agt,P),bel(Agt,goal(Speaker,believe(Agt,P,1,T),0.7),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,remind(Speaker,Agt,P),bel(Agt,believe(Speaker,P,1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
%--
%--
%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%
%-> Commissive MCA %
%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%
%-> Promise
%%%%%%%%%
test

% send_conditions
%old%bel(Agt,before(Speaker,promise(Speaker,Hearer,P),[ground(Hearer), [bel(Agt,goal(Speaker,believe(Hearer,goal(Speaker,resp(Speaker,P,_T),_D1),_D2,_T1),D3),_Dbp,_Tbp), D3<0.75]]),1,0,speech_rule) :- current_agent(Agt).
%old%bel(Agt,before(Speaker,promise(Speaker,Hearer,P),[ground(Hearer), [bel(Agt,goal(Speaker,resp(Speaker,P,_T),_D1),_Dbp,_Tbp)]]),1,0,speech_rule) :- current_agent(Agt).
%new%bel(Agt,before(Speaker,promise(Speaker,Hearer,P),[bel(Agt,believe(Speaker,goal(Hearer,P,_D5),_D6,_T),_Dbp,_Tbp),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).
send_condition(speechact(promise,Speaker,Hearer,P),[believe(Speaker,goal(Hearer,P,_D5),_Db,T)]) :- instant(T).

% interpret conseq
%bel(Agt,instant(T)->after(Agt,promise(Agt,Hearer,P),bel(Agt,committed(Agt,Hearer,P),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
send_effects(speechact(promise,Speaker,Hearer,P),committed(Speaker,Hearer,P)) :- instant(T).

%bel(Agt,instant(T)->after(Agt,promise(Agt,Hearer,P),bel(Agt,believe(Hearer, goal(Agt,resp(Agt,P,T),0.7),1,T),1.0,T,communication)),1,0,speech_rule) :- current_agent(Agt).
send_effects(speechact(promise,Speaker,Hearer,P),believe(Hearer,goal(Speaker,resp(Speaker,P,T),0.7),1,T)) :- instant(T).

% receive conseq
bel(Agt,instant(T)->after(Speaker,promise(Speaker,Agt,P),bel(Agt,goal(Speaker,resp(Speaker,P,T),0.7),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,promise(Speaker,Agt,P),bel(Agt,believe(Speaker,goal(Agt,P,0.7),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).


%--
%%%%%%%%%

%%%%%%%%%
%-> Assure
%%%%%%%%%

% send_conditions
%bel(Agt,before(Speaker,assure(Speaker,Hearer,P),[ground(Hearer), [bel(Agt,goal(Speaker,believe(Hearer,goal(Speaker,resp(Speaker,P,_T),_D1),_D2,_T1),D3),_Dbp,_Tbp),D3>=0.75]]),1,0,speech_rule) :- current_agent(Agt).
%bel(Agt,before(Speaker,assure(Speaker,Hearer,P),[ground(Hearer), [bel(Agt,goal(Speaker,resp(Speaker,P,_T),_D1),_Dbp,_Tbp)]]),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,before(Speaker,assure(Speaker,Hearer,P),[bel(Agt,believe(Speaker,goal(Hearer,P,_D5),_D6,_T),_Dbp,_Tbp),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,assure(Agt,Hearer,P),bel(Agt,committed(Agt,Hearer,P),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Agt,assure(Agt,Hearer,P),bel(Agt,believe(Hearer, goal(Agt,resp(Agt,P,T),0.7),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,assure(Speaker,Agt,P),bel(Agt,goal(Speaker,resp(Speaker,P,T),0.8),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,assure(Speaker,Agt,P),bel(Agt,believe(Speaker,goal(Agt,P,0.7),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,assure(Speaker,Agt,P),bel(Agt,goal(Speaker,believe(Agt,goal(Speaker,resp(Speaker,P,T),0.8),1,T),0.7),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

%--
%%%%%%%%%

%%%%%%%%%
%-> Accept_demand
%%%%%%%%%
% accept an action performed by self

%send_conditions
bel(Agt,before(Speaker,accept_demand(Speaker,Hearer,P),[bel(Agt,done(Hearer,ask(Hearer,Speaker,P),_T),_D,_Td),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).
% bel(Agt,before(Speaker,accept_demand(Speaker,Hearer,P),[bel(Agt,believe(Speaker,goal(Hearer,resp(Speaker,P,_T),_D1),_D2,_T2),_Dbp,_Tbp),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).
% bel(Agt,before(Speaker,accept_demand(Speaker,Hearer,P),[bel(Agt,goal(Speaker,resp(Speaker,P,_T),_D4),_Dbp,_Tbp),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,before(Speaker,accept_demand(Speaker,Hearer,P),[not(bel(Agt,goal(Speaker,not(resp(Speaker,P,_Tr),_Dr)),_D,_T)),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,accept_demand(Agt,Hearer,P),bel(Agt,committed(Agt,Hearer,P),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,accept_demand(Speaker,Agt,P),bel(Agt,believe(Speaker,goal(Agt,resp(Speaker,P,T),0.7),0.75,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,accept_demand(Speaker,Agt,P),bel(Agt,goal(Speaker,resp(Speaker,P,T),0.7),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

%--
%%%%%%%%%

%%%%%%%%%
%-> Accept_offer
%%%%%%%%%

% accept an action performed by the Hearer

%send_conditions
bel(Agt,before(Speaker,accept_offer(Speaker,Hearer,P),[bel(Agt,done(Hearer,offer(Hearer,Speaker,P),_T),_D,_Td),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).
% bel(Agt,before(Speaker,accept_offer(Speaker,Hearer,P),[bel(Agt,believe(Speaker,goal(Hearer,resp(Hearer,P,_T),_D1),_D2,_T2),_Dbp,_Tbp),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).
% bel(Agt,before(Speaker,accept_offer(Speaker,Hearer,P),[bel(Agt,goal(Speaker,resp(Hearer,P,_T),_D4),_Dbp,_Tbp),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,accept_offer(Agt,Hearer,P),bel(Agt,committed(Hearer,Agt,P),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,accept_offer(Speaker,Agt,P),bel(Agt,believe(Speaker,goal(Agt,resp(Agt,P,T),0.7),0.75,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,accept_offer(Speaker,Agt,P),bel(Agt,goal(Speaker,resp(Agt,P,T),0.7),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,accept_offer(Speaker,Agt,P),bel(Agt,committed(Agt,Speaker,P),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).


%--
%%%%%%%%%

%%%%%%%%%
%-> Refuse
%%%%%%%%%

%send_conditions
bel(Agt,before(Speaker,refuse(Speaker,Hearer,P),[bel(Agt,believe(Speaker,goal(Hearer,resp(Speaker,P,_T),_D1),_D2,_T1),_Dbp,_Tbp),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).
%bel(Agt,before(Speaker,refuse(Speaker,Hearer,P),[bel(Agt,notgoal(Speaker,resp(Speaker,P,_T),_D4,_T4,_S4),_Dbp,_Tbp),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,refuse(Agt,Hearer,P),bel(Agt,believe(Hearer,goal(Agt,not(resp(Agt,P,_T)),_D4),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Agt,refuse(Agt,_Hearer,P),bel(Agt,resp(Agt,not(P),T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,refuse(Speaker,Agt,P),bel(Agt,believe(Speaker,goal(Agt,resp(Speaker,P,T),0.7),0.75,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,refuse(Speaker,Agt,P),bel(Agt,goal(Speaker,not(resp(Speaker,P,T),0.7)),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,refuse(Speaker,Agt,P),bel(Agt,resp(Speaker,not(P),T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).



%--
%%%%%%%%%

%%%%%%%%%
%-> Offer
%%%%%%%%%

%send_conditions
bel(Agt,before(Speaker,offer(Speaker,Hearer,P),[bel(Agt,goal(Speaker,resp(Speaker,P,_T),_D1),_Dbp,_Tbp),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,before(Speaker,offer(Speaker,Hearer,P),[not(bel(Agt,believe(Speaker, goal(Hearer, resp(Speaker,P,_T), _D2), _D3, _T3),_Dbp,_Tbp)),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).
%bel(Agt,before(Speaker,offer(Speaker,Hearer,P),[bel(Agt,believe(Speaker,goal(Hearer,P,_D6),_D7,_T),_Dbp,_Tbp),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,offer(Agt,Hearer,P),bel(Agt,believe(Hearer,goal(Agt,resp(Agt,P,_T),0.8),1,_T1),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,offer(Speaker,Agt,P),bel(Agt,goal(Speaker,resp(Speaker,P,T),0.7),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,offer(Speaker,Agt,P),bel(Agt,not(believe(Speaker,goal(Agt,resp(Speaker,P,T),1),0.75,T)),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,offer(Speaker,Agt,P),bel(Agt,believe(Speaker,goal(Agt,P,1),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
%--
%%%%%%%%%

%%%%%%%%%
%-> Renounce
%%%%%%%%%

% send_conditions
bel(Agt,before(Speaker,renounce(Speaker,Hearer,P),[bel(Agt,notgoal(Speaker,resp(Speaker,P,_T),_D1,_T1,_S1),_Dbp,_Tbp),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).
%bel(Agt,before(Speaker,renounce(Speaker,Hearer,P),[ground(Hearer), [bel(Agt,goal(Speaker,believe(Hearer, not(goal(Speaker,resp(Speaker,P,_T),_D1)), _D2, _T2), _D3),_Dbp,_Tbp)]]),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,before(Speaker,renounce(Speaker,Hearer,P),[bel(Agt,believe(Speaker,commited(Speaker,Hearer,P),_D,_T),_Dbp,_Tbp),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,renounce(Agt,Hearer,P),bel(Agt,not(committed(Agt,Hearer,P)),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,renounce(Speaker,Agt,P),bel(Agt,goal(Speaker,not(resp(Speaker,P,T),0.7)),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,renounce(Speaker,Agt,P),bel(Agt,not(committed(Speaker,Agt,P)),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

%--
%%%%%%%%%
%--
%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%
%-> Directive MCA %%
%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%
%-> Ask
%%%%%%%%%

% send_conditions
bel(Agt,before(Speaker,ask(Speaker,Hearer,P),[bel(Agt,goal(Speaker,resp(Hearer,P,_T),D1),_Dbp,_Tbp),[ground(Hearer), D1>=0.5, D1<0.75]]),1,0,speech_rule) :- current_agent(Agt).
%bel(Agt,before(Speaker,ask(Speaker,Hearer,P),[ground(Hearer), [bel(Agt,goal(Speaker,believe(Hearer, goal(Speaker,resp(Hearer,P,_T),D1),_D2,_T1),_D3),_Dbp,_Tbp), D1>=0.5, D1<0.75]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,ask(Agt,Hearer,P),bel(Agt,believe(Hearer,goal(Agt,resp(Hearer,P,_T),0.7),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,ask(Speaker,Agt,P),bel(Agt,goal(Speaker,resp(Agt,P,T),0.6),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,ask(Speaker,Agt,P),bel(Agt,goal(Speaker,believe(Agt,goal(Speaker,resp(Agt,P,T),0.6),1,T),0.6),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

%-- 
%%%%%%%%%

%%%%%%%%%
%-> Ask_val
%%%%%%%%%

% send_conditions
%bel(Agt, before(Speaker,ask_val(Speaker,Hearer,P),[ground(Hearer),[bel(Agt,goal(Speaker,knowval(Speaker,P,_),_D1),_Dbp,_Tbp)]]),1,0,speech_rule) :- current_agent(Agt).
bel(Agt, before(Speaker,ask_val(Speaker,Hearer,P),[bel(Speaker,believe(Speaker,knowval(Hearer,P,_),_D1,_T),_Dbp,_Tbp),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).
bel(Agt, before(Speaker,ask_val(Speaker,Hearer,P),[not(bel(Speaker,believe(Speaker,knowval(Speaker,P,_),_D1,_T),_Dbp,_Tbp)),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
%% cooperative user supposition
bel(Agt,instant(T)->after(Agt,ask_val(Agt,Hearer,P),bel(Agt,goal(Hearer,knowval(Agt,P,_),1),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Agt,ask_val(Agt,Hearer,P),bel(Agt,believe(Hearer,goal(Agt,knowval(Agt,P,_),1),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Agt,ask_val(Agt,Hearer,P),bel(Agt,not(believe(Hearer,knowval(Agt,P,_),1,T)),1,T,communication)),1,0,speech_rule) :- current_agent(Agt). 

% receive conseq
bel(Agt,instant(T)->after(Speaker,ask_val(Speaker,Agt,P),bel(Agt,goal(Speaker,knowval(Speaker,P,_),1),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,ask_val(Speaker,Agt,P),bel(Agt,not(believe(Speaker,knowval(Speaker,P,_),1,T)),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,ask_val(Speaker,Agt,P),bel(Agt,goal(Speaker,resp(Agt,knowval(Speaker,P,_),T),0.6),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

%--
%%%%%%%%%

%%%%%%%%%
%-> Demand
%%%%%%%%%

% send_conditions
bel(Agt,before(Speaker,demand(Speaker,Hearer,P),[bel(Agt,goal(Speaker,resp(Hearer,P,_T),D1),_Dbp,_Tbp),[ground(Hearer), D1>=0.75]]),1,0,speech_rule) :- current_agent(Agt).
%bel(Agt,before(Speaker,demand(Speaker,Hearer,P),[ground(Hearer), [bel(Agt,goal(Speaker,believe(Hearer, goal(Speaker,resp(Hearer,P,_T),D1),_D2,_T1),_D3),_Dbp,_Tbp), D1>=0.75]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,demand(Agt,Hearer,P),bel(Agt,believe(Hearer,goal(Agt,resp(Hearer,P,_T),0.8),_D2,_T1),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,demand(Speaker,Agt,P),bel(Agt,goal(Speaker,resp(Agt,P,T),0.8),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,demand(Speaker,Agt,P),bel(Agt,goal(Speaker,believe(Agt,goal(Speaker,resp(Agt,P,T),0.8),1,T),0.8),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

%-- 
%%%%%%%%%

%%%%%%%%%
%-> Advise
%%%%%%%%%

% send_conditions
bel(Agt,before(Speaker,advise(Speaker,Hearer,P),[bel(Agt,goal(Speaker,resp(Hearer,P,_T),D1),_Dbp,_Tbp),[ground(Hearer), D1>=0.25, D1<0.5]]),1,0,speech_rule) :- current_agent(Agt).
%bel(Agt,before(Speaker,advise(Speaker,Hearer,P),[ground(Hearer), [bel(Agt,goal(Speaker,believe(Hearer, goal(Speaker,resp(Hearer,P,_T),D1),_D2,_T1),_D3),_Dbp,_Tbp), D1>=0.25, D1<0.5]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,advise(Agt,Hearer,P),bel(Agt,believe(Hearer,goal(Agt,resp(Hearer,P,_T),0.4),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,advise(Speaker,Agt,P),bel(Agt,goal(Speaker,resp(Agt,P,T),0.4),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,advise(Speaker,Agt,P),bel(Agt,goal(Speaker,believe(Agt,goal(Speaker,resp(Agt,P,T),0.4),1,T),0.4),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

%-- 
%%%%%%%%%

%%%%%%%%%
%-> Suggest
%%%%%%%%%

% send_conditions
bel(Agt,before(Speaker,suggest(Speaker,Hearer,P),[bel(Agt,goal(Speaker,resp(Hearer,P,_T),D1),_Dbp,_Tbp),[ground(Hearer), D1<0.25]]),1,0,speech_rule) :- current_agent(Agt).
%bel(Agt,before(Speaker,suggest(Speaker,Hearer,P),[ground(Hearer), [bel(Agt,goal(Speaker,believe(Hearer, goal(Speaker,resp(Hearer,P,_T),D1),_D2,_T1),_D3),_Dbp,_Tbp), D1<0.25]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,suggest(Agt,Hearer,P),bel(Agt,believe(Hearer,goal(Agt,resp(Hearer,P,_T),0.2),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,suggest(Speaker,Agt,P),bel(Agt,goal(Speaker,resp(Agt,P,T),0.2),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,suggest(Speaker,Agt,P),bel(Agt,goal(Speaker,believe(Agt,goal(Speaker,resp(Agt,P,T),0.2),1,T),0.2),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

%-- 
%%%%%%%%%
%-- 
%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%
%-> Expressive MCA %
%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%
%-> Greet %%
%%%%%%%%%%%%
% send_conditions
bel(Agt,before(Speaker,greet(Speaker,Hearer,saw(Speaker,Hearer)),[not(believe(Speaker,believe(Hearer,saw(Speaker,Hearer),_D,_T1),_D1,_T2)),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,before(Speaker,greet(Speaker,Hearer,saw(Speaker,Hearer)),[feel_emotion(Speaker,joy,saw(Speaker,Hearer),_D1,_T1),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,greet(Agt,Hearer,saw(Agt,Hearer)),bel(Agt,believe(Hearer,saw(Agt,Hearer),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Agt,greet(Agt,Hearer,saw(Agt,Hearer)),bel(Agt,believe(Hearer,feel_emotion(Agt,joy,saw(Agt,Hearer),1,T),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,greet(Speaker,Agt,saw(Speaker,Agt)),bel(Agt,goal(Speaker,saw(Speaker,Agt),1),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,greet(Speaker,Agt,saw(Speaker,Agt)),bel(Agt,saw(Speaker,Agt),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,greet(Speaker,Agt,saw(Speaker,Agt)),bel(Agt,saw(Agt,Speaker),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

%--
%%%%%%%%%%%%


%%%%%%%%%%%%
%-> Cry %%
%%%%%%%%%%%%
% send_conditions
bel(Agt,before(Speaker,cry(Speaker,Hearer,P),[feel_emotion(Speaker,sadness,P,_D1,_T1),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,cry(Agt,Hearer,P),bel(Agt,believe(Hearer,feel_emotion(Agt,sadness,P,0.7,T),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,cry(Speaker,Agt,P),bel(Agt,goal(Speaker,not(P),0.8),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

%--
%%%%%%%%%%%%

%%%%%%%%%%%%
%-> Approve %%
%%%%%%%%%%%%
% send_conditions
bel(Agt,before(Speaker,approve(Speaker,Hearer,P),[feel_emotion(Speaker,approval,P,_D1,_T1)],[ground(Hearer)]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,approve(Agt,Hearer,P),bel(Agt,believe(Hearer,feel_emotion(Agt,approval,P,0.7,T),0.8,T),0.8,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq

bel(Agt,instant(T)->after(Speaker,approve(Speaker,Agt,P),bel(Agt,ideal(Speaker,P,0.8),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
%--
%%%%%%%%%%%%

%%%%%%%%%%%%
%-> Disapprove %%
%%%%%%%%%%%%
% send_conditions
bel(Agt,before(Speaker,disapprove(Speaker,Hearer,P),[feel_emotion(Speaker,disapproval,P,_D1,_T1),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,disapprove(Agt,Hearer,P),bel(Agt,believe(Hearer,feel_emotion(Agt,disapproval,P,0.7,T),0.8,T),0.8,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,disapprove(Speaker,Agt,P),bel(Agt,ideal(Speaker,not(P),0.8),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
%--

%%%%%%%%%%%%



%%%%%%%%%%%%%%
%-> Rejoice %%
%%%%%%%%%%%%%%

% send_conditions
bel(Agt,before(Speaker,rejoice(Speaker,Hearer,P), [feel_emotion(Speaker,rejoicing,P,D1,_T1),[ground(Hearer),D1<0.75]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,rejoice(Agt,Hearer,P),bel(Agt,believe(Hearer,feel_emotion(Agt,rejoicing,P,0.7,T),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,rejoice(Speaker,Agt,P),bel(Agt,goal(Speaker,P,0.7),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,rejoice(Speaker,Agt,P),bel(Agt,believe(Speaker,resp(Speaker,P,T),0.75,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
%--
%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%
%-> Be Very Pleased %%
%%%%%%%%%%%%%%%%%%%%%%

% send_conditions
bel(Agt,before(Speaker,be_very_pleased(Speaker,Hearer,P), [feel_emotion(Speaker,rejoicing,P,D1,_T),[ground(Hearer), D1>=0.75]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,be_very_pleased(Agt,Hearer,P),bel(Agt,believe(Hearer,feel_emotion(Agt,rejoicing,P,0.9,T),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,be_very_pleased(Speaker,Agt,P),bel(Agt,goal(Speaker,P,0.9),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,be_very_pleased(Speaker,Agt,P),bel(Agt,believe(Speaker,resp(Speaker,P,T),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
%--
%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%
%-> Thank %%
%%%%%%%%%%%%

% send_conditions
bel(Agt,before(Speaker,thank(Speaker,Hearer,P), [feel_emotion(Speaker,gratitude(Hearer),P,D1,_T),[ground(Hearer),D1<0.75]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,thank(Agt,Hearer,P),bel(Agt,believe(Hearer,feel_emotion(Agt,gratitude(Hearer),P,0.7,T),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,thank(Speaker,Agt,P),bel(Agt,goal(Speaker,P,0.7),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,thank(Speaker,Agt,P),bel(Agt,believe(Speaker,resp(Agt,P,T),0.75,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
%--
%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%
%-> Congratulate %%
%%%%%%%%%%%%%%%%%%%

% send_conditions
bel(Agt,before(Speaker,congratulate(Speaker,Hearer,P), [feel_emotion(Speaker,gratitude(Hearer),P,D1,_T1),[ground(Hearer), D1>=0.75]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,congratulate(Agt,Hearer,P),bel(Agt,believe(Hearer,feel_emotion(Agt,gratitude(Hearer),P,0.9,T),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,congratulate(Speaker,Agt,P),bel(Agt,goal(Speaker,P,0.9),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,congratulate(Speaker,Agt,P),bel(Agt,believe(Speaker,resp(Agt,P,T),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
%--
%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%
%-> Regret %%
%%%%%%%%%%%%%

% send_conditions
bel(Agt,before(Speaker,regret(Speaker,Hearer,P), [feel_emotion(Speaker,regret,not(P),D1,_T1),[ground(Hearer), D1<0.75]]),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,before(Speaker,regret(Speaker,Hearer,P), [not(believe(Speaker,goal(Hearer,P,_D2),_D3,_T)),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,regret(Agt,Hearer,P),bel(Agt,believe(Hearer,feel_emotion(Agt,regret,not(P),0.7,T),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,regret(Speaker,Agt,P),bel(Agt,goal(Speaker,not(P),0.7),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,regret(Speaker,Agt,P),bel(Agt,believe(Speaker,resp(Speaker,P,T),0.75,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
%--
%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%
%-> Apologize %%
%%%%%%%%%%%%%%%%

% send_conditions
bel(Agt,before(Speaker,apologize(Speaker,Hearer,P), [feel_emotion(Speaker,regret,not(P),_D1,_T1),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,before(Speaker,apologize(Speaker,Hearer,P), [believe(Speaker,goal(Hearer,P,_D2),_D3,_T),[ground(Hearer)]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,apologize(Agt,Hearer,P),bel(Agt,believe(Hearer,feel_emotion(Agt,regret,not(P),0.7,T),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,apologize(Speaker,Agt,P),bel(Agt,goal(Speaker,not(P),0.7),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,apologize(Speaker,Agt,P),bel(Agt,believe(Speaker,resp(Speaker,P,T),0.6,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,apologize(Speaker,Agt,P),bel(Agt,believe(Speaker,goal(Agt,not(P),0.7),0.75,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
%--
%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%
%-> Beg Pardon %%
%%%%%%%%%%%%%%%%%

% send_conditions
bel(Agt,before(Speaker,beg_pardon(Speaker,Hearer,P), [feel_emotion(Speaker,regret,not(P),D1,_T1),[ground(Hearer), D1>=0.75]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,beg_pardon(Agt,Hearer,P),bel(Agt,believe(Hearer,feel_emotion(Agt,regret,not(P),0.9,T),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,beg_pardon(Speaker,Agt,P),bel(Agt,goal(Speaker,not(P),0.9),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,beg_pardon(Speaker,Agt,P),bel(Agt,believe(Speaker,resp(Speaker,P,T),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
%--
%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%
%-> Complain %%
%%%%%%%%%%%%%%%

% send_conditions
bel(Agt,before(Speaker,complain(Speaker,Hearer,P), [feel_emotion(Speaker,disappointment(Hearer),P,D1,_T1),[ground(Hearer), D1<0.75]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,complain(Agt,Hearer,P),bel(Agt,believe(Hearer,feel_emotion(Agt,disappointment(Hearer),P,0.7,T),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,complain(Speaker,Agt,P),bel(Agt,goal(Speaker,not(P),0.7),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,complain(Speaker,Agt,P),bel(Agt,believe(Speaker,resp(Agt,P,T),0.75,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
%--
%%%%%%%%%%%%%%%

%%%%%%%%%%%
%-> Moan %%
%%%%%%%%%%%

% send_conditions
bel(Agt,before(Speaker,moan(Speaker,Hearer,P), [feel_emotion(Speaker,disappointment(Hearer),P,D1,_T1),[ground(Hearer), D1>=0.75]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,moan(Agt,Hearer,P),bel(Agt,believe(Hearer,feel_emotion(Agt,disappointment(Hearer),P,0.9,T),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,moan(Speaker,Agt,P),bel(Agt,goal(Speaker,not(P),0.9),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,moan(Speaker,Agt,P),bel(Agt,believe(Speaker,resp(Agt,P,T),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
%--
%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%
%-> Feel Guilty %%
%%%%%%%%%%%%%%%%%%

% send_conditions
bel(Agt,before(Speaker,feel_guilty(Speaker,Hearer,P), [feel_emotion(Speaker,guilt,P,D1,_T1),[ground(Hearer), D1<0.75]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,feel_guilty(Agt,Hearer,P),bel(Agt,believe(Hearer,feel_emotion(Agt,guilt,P,0.7,T),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,feel_guilty(Speaker,Agt,P),bel(Agt,ideal(Speaker,P,0.7),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,feel_guilty(Speaker,Agt,P),bel(Agt,believe(Speaker,resp(Speaker,P,T),0.75,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
%--
%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%
%-> Suffer For %%
%%%%%%%%%%%%%%%%%

% send_conditions
bel(Agt,before(Speaker,suffer_for(Speaker,Hearer,P), [feel_emotion(Speaker,guilt,P,D1,_T1),[ground(Hearer), D1>=0.75]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,suffer_for(Agt,Hearer,P),bel(Agt,believe(Hearer,feel_emotion(Agt,guilt,P,0.9,T),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,suffer_for(Speaker,Agt,P),bel(Agt,ideal(Speaker,P,0.9),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,suffer_for(Speaker,Agt,P),bel(Agt,believe(Speaker,resp(Speaker,P,T),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
%--
%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%
%-> Reproach %%%%%
%%%%%%%%%%%%%%%%%%

% send_conditions
bel(Agt,before(Speaker,reproach(Speaker,Hearer,P), [feel_emotion(Speaker,reproach(Hearer),P,D1,_T1),[ground(Hearer), D1<0.75]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,reproach(Agt,Hearer,P),bel(Agt,believe(Hearer,feel_emotion(Agt,reproach(Hearer),P,0.7,T),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,reproach(Speaker,Agt,P),bel(Agt,ideal(Speaker,P,0.7),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,reproach(Speaker,Agt,P),bel(Agt,believe(Speaker,resp(Speaker,P,T),0.75,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
%--
%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%
%-> Protest %%%%%
%%%%%%%%%%%%%%%%%

% send_conditions
bel(Agt,before(Speaker,protest(Speaker,Hearer,P), [feel_emotion(Speaker,reproach(Hearer),P,D1,_T1),[ground(Hearer), D1>=0.75]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,protest(Agt,Hearer,P),bel(Agt,believe(Hearer,feel_emotion(Agt,reproach(Hearer),P,0.9,T),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,protest(Speaker,Agt,P),bel(Agt,ideal(Speaker,P,0.9),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,protest(Speaker,Agt,P),bel(Agt,believe(Speaker,resp(Speaker,P,T),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
%--
%%%%%%%%%%%%%%

%%%%%%%%%%%%%%
%-> Satisfy %%
%%%%%%%%%%%%%%

% send_conditions
bel(Agt,before(Speaker,satisfy(Speaker,Hearer,P), [feel_emotion(Speaker,moral_satisfaction,P,D1,_T1),[ground(Hearer), D1<0.75]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,satisfy(Agt,Hearer,P),bel(Agt,believe(Hearer,feel_emotion(Agt,moral_satisfaction,P,0.7,T),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,satisfy(Speaker,Agt,P),bel(Agt,ideal(Speaker,P,0.7),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,satisfy(Speaker,Agt,P),bel(Agt,believe(Speaker,resp(Speaker,P,T),0.75,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
%--
%%%%%%%%%%%%%%

%%%%%%%%%%%%
%-> Boast %%
%%%%%%%%%%%%

% send_conditions
bel(Agt,before(Speaker,boast(Speaker,Hearer,P), [feel_emotion(Speaker,moral_satisfaction,P,D1,_T1),[ground(Hearer), D1>=0.75]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,boast(Agt,Hearer,P),bel(Agt,believe(Hearer,feel_emotion(Agt,moral_satisfaction,P,0.9,T),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,boast(Speaker,Agt,P),bel(Agt,ideal(Speaker,P,0.9),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,boast(Speaker,Agt,P),bel(Agt,believe(Speaker,resp(Speaker,P,T),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
%--
%%%%%%%%%%%%

%%%%%%%%%%%%%%%%
%-> Subscribe %%
%%%%%%%%%%%%%%%%

% send_conditions
bel(Agt,before(Speaker,subscribe(Speaker,Hearer,P), [feel_emotion(Speaker,adhesion(Hearer),P,D1,_T1),[ground(Hearer), D1<0.75]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,subscribe(Agt,Hearer,P),bel(Agt,believe(Hearer,feel_emotion(Agt,adhesion(Hearer),P,0.7,T),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,subscribe(Speaker,Agt,P),bel(Agt,ideal(Speaker,P,0.7),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,subscribe(Speaker,Agt,P),bel(Agt,believe(Speaker,resp(Agt,P,T),0.75,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
%--
%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%
%-> Compliment %%
%%%%%%%%%%%%%%%%%

% send_conditions
bel(Agt,before(Speaker,compliment(Speaker,Hearer,P), [feel_emotion(Speaker,adhesion(Hearer),P,D1,_T1),[ground(Hearer), D1>=0.75]]),1,0,speech_rule) :- current_agent(Agt).

% interpret conseq
bel(Agt,instant(T)->after(Agt,compliment(Agt,Hearer,P),bel(Agt,believe(Hearer,feel_emotion(Agt,adhesion(Hearer),P,0.9,T),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).

% receive conseq
bel(Agt,instant(T)->after(Speaker,compliment(Speaker,Agt,P),bel(Agt,ideal(Speaker,P,0.9),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
bel(Agt,instant(T)->after(Speaker,compliment(Speaker,Agt,P),bel(Agt,believe(Speaker,resp(Agt,P,T),1,T),1,T,communication)),1,0,speech_rule) :- current_agent(Agt).
%--
%%%%%%%%%%%%%%%%%

%--
%%%%%%%%%%%%%%%%%%%%


