

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DECLARATIONS of          %%
%% discontiguous predicates %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

:- discontiguous(drule/5).
:- discontiguous(drulec/5).

:- discontiguous(illoc_point/3).
:- discontiguous(achiev_mode_effect/3).
:- discontiguous(preparatory_cond/3).
:- discontiguous(sincerity_cond/3).

:- discontiguous(hearer/2).
:- discontiguous(speaker/2).
:- discontiguous(illoc_force/2).
:- discontiguous(prop_content/2).

:- discontiguous(speech_act/2).

:- [pleiad_semantics_benoit],[pleiad_grounding],[pleiad_trust_yves].


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% new APRIL 2013 : rules for effects of speech acts       %%
%% coming from axioms defined in AAMAS 2012                %%
%% the drules are defined in pleiad_speech_acts_benoit.pl  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% TODO: check if defined
% not(predicate_property(drule(_,_,_,_),undefined))

% dialogue rules rule(25)
rule(Index,T,Agt,Prem,Cci) :-
    drule(I,T,Agt,Prem,Cci),Index is 250+I.

% consistency rule: extra consistency condition - rule(26)
rule(Index,T,Agt,Prem,grounded(K,Phi)) :-
    drulec(I,T,Agt,Prem,grounded(K,Phi)), Index is 260+I,
    not(grounded(K,not(Phi))).
    
% planning rules - adopt intention to perform speech act that reaches goal,
% independently of any conditions  rule(27)
rule(Index,T,Agt,Prem,Cci) :-
    prule(I,T,Agt,Prem,Cci),Index is 270+I.

% trust rules defined in pleiad_trust.pl

% obligation operator: impersonal : time will be in the belief, agent in the prop=done(agent,alpha)
% believe(Agt,oblig(Prop))
% capable operator
% capable(Agent,Action) dans un bel


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% GENERAL RULES FOR HANDLING SPEECH ACTS  %%
%% correspond to AAMAS axioms              %%
%% imported in pleiad_demo rules           %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% drule(T,agt,prem,cci). = rule(250)
%% force(speaker,hearer,propcontent)

%% WARNING: premises of rules have to be mental attitudes of the agent

%%%%%%%%%
% AXIOM 0 : utterance at will
% choice happens alpha-speechact top -> happens alpha-loc top
%%%%%%%%%

% general rule for intention to perform a speech act
% intend(Agt,done(Agt,SpeechAct,_),_Deg,_Time) :- cond_spact(Agt,SpeechAct).
% NO : cannot adopt intention to do everything just because it is doable
%       motivation for performing a speech act needs to be external

% from goal to perform illocutionary act, deduce goal to utter words that reach this goal
drule(1,T,Agt,
      % premise
      ([
        goal(Agt,done(Agt,IllocAct,Tact),Deg,T,S),
        speech_act(IllocAct,LocAct)
      ]),
      % conclusion: intention at same instant and with same deg
      % to perform locutionary act in an attempt to perform illoc act
        goal(Agt,done(Agt,LocAct,Tact),Deg,T,S)
      ).

% axiom0 - happens ou done ???
% happens(Agt,SpeechAct) :-    intend(Agt,SpeechAct).

% link between a speech act and its locutionary act
% selection of one possible locutionary act among potentially several possibilities

%%%%%%%%%%
% AXIOM 1 : expressed mental states
% done alpha-loc top -> grd(i,j) done alpha-loc intend-i (illocpt and acvmdeff)
%%%%%%%%%%

% consistency rule
% it is grounded that the locutionary act was performed
drulec(1,T,Agt,
       % premise
       ([
        % the locutionary act was performed at instant T
        %%T1 is T-1,
        believe(Agt,done(Speaker,say(Speaker,Hearer,WordsLoc),T),_DegB,T)  %%done(T1)
       ]),
       % conclusion 1: grounded that loc-act was performed
       grounded([Speaker,Hearer],done(Speaker,say(Speaker,Hearer,WordsLoc),T))  %%T1
).

% and it is grounded that before it was performed the speaker
% had the intention to obtain its illoc point 
drulec(2,T,Agt,
       % premise
       ([
        % the locutionary act was performed at instant T
        %%T1 is T-1,
        believe(Agt,done(Speaker,say(Speaker,Hearer,WordsLoc),T),_DegB,T),
        % and at the moment when it was performed 
        % the corresponding illocutionary act
        speech_act(IllocAct,say(Speaker,Hearer,WordsLoc)),
        % with given illoc point
        illoc_point(T,IllocAct,IllocPoint)  %%T1
       ]),
       % conclusion 2
       grounded([Speaker,Hearer],intend(Speaker,IllocPoint,T))   %%T1
).

% and the intention to obtain achievement mode effect
drulec(3,T,Agt,
       % premise
       ([
        % the locutionary act was performed at instant T
        %% T1 is T-1,
        believe(Agt,done(Speaker,say(Speaker,Hearer,WordsLoc),T),_DegB,T),  %%done(T1)
        % and at the time of the performance
        % the corresponding illocutionary act
        speech_act(IllocAct,say(Speaker,Hearer,WordsLoc)),
        % with given achievement mode effect 
        achiev_mode_effect(T,IllocAct,AcvMdEff)   %%T1
       ]),
       % conclusion 3
       grounded([Speaker,Hearer],intend(Speaker,AcvMdEff,T))   %%T1
).

% and the belief of prep cond
drulec(4,T,Agt,
       % premise
       ([
        % the locutionary act was performed at instant T
        %% T1 is T-1,
        believe(Agt,done(Speaker,say(Speaker,Hearer,WordsLoc),T),_DegB,T),  %%done(T1)
        % and at time of the performance
        % the corresponding illocutionary act
        speech_act(IllocAct,say(Speaker,Hearer,WordsLoc)),
        % with given preparatory cond 
        preparatory_cond(T,IllocAct,PrepCd)  %%T1
       ]),
       % conclusion 4: grounded that speaker believed the prep cond when performing the loc act
       grounded([Speaker,Hearer],believe(Speaker,PrepCd,1,T))   %%T1
).

% and the sincerity cond
drulec(5,T,Agt,
       % premise
       ([
        % the locutionary act was performed at instant T-1
        %T1 is T-1,
        believe(Agt,done(Speaker,say(Speaker,Hearer,WordsLoc),T),_DegB,T),  %done(T1)
        % and at the time of performance
        % the corresponding illocutionary act
        speech_act(IllocAct,say(Speaker,Hearer,WordsLoc)),
        % with given sincerity cond 
        sincerity_cond(T,IllocAct,SincCd)    %T1
       ]),
       % conclusion 5 : grounded sincerity condition
       grounded([Speaker,Hearer],SincCd)
).



%%%%%%
%% AXIOM 2
%% deduced effects
%%%%%%

% illoc point
drule(2,T,Agt,
    % premises = conclus of axiom 1
    ([
        % alpha-loc was done at T
        %%T1 is T-1,
        believe(Agt,done(Speaker,AlphaLoc,T),_DegB,T),   %%done(T1)
        % it is grounded it was done at instant T1 (from ax1a)
        is_grounded([Speaker,Hearer],done(Speaker,AlphaLoc,T)),   %%T1
        % corresponding illocutionary act
        speech_act(AlphaIlloc,AlphaLoc),
        % its sincerity cond at time of perf
        sincerity_cond(T,AlphaIlloc,SincCd),  %%T1
        % and its preparatory condition at time of perf
        preparatory_cond(T,AlphaIlloc,PrepCd),  %%T1
        % it is grounded they were true at time of performance of loc act
        is_grounded([Speaker,Hearer],SincCd),
        is_grounded([Speaker,Hearer],believe(Speaker,PrepCd,_Deg,T)),  %%T1
        % get its illocutionary point now at current instant
        illoc_point(T,AlphaIlloc,IllocPt)
    ]),
    % conclu = illocutionary point is true
    IllocPt
).


%% ground(illoc point)
% FIXME : for now deduce both illoc point and ground(illoc point) because no demo rules allow to deduce it automatically
drulec(6,T,Agt,
    % premises = conclus of axiom 1
    ([
        % alpha-loc was done
        %%T1 is T-1,
        believe(Agt,done(Speaker,AlphaLoc,T),_DegB,T),   %%done(T1)
        % it is grounded it was done at instant T (ax1a)
        is_grounded([Speaker,Hearer],done(Speaker,AlphaLoc,T)),   %%T1
        % corresponding illocutionary act
        speech_act(AlphaIlloc,AlphaLoc),
        % its sincerity cond at time of perf
        sincerity_cond(T,AlphaIlloc,SincCd),   %%T1
        % and its preparatory condition at perf time
        preparatory_cond(T,AlphaIlloc,PrepCd),   %%T1
        % it is grounded they were true at time of perf
        is_grounded([Speaker,Hearer],SincCd),
        is_grounded([Speaker,Hearer],believe(Speaker,PrepCd,_Deg,T)),   %%T1
        % get its illocutionary point at current time
        illoc_point(T,AlphaIlloc,IllocPt)
    ]),
    % conclu = illocutionary point is also grounded (directly)
    grounded([Speaker,Hearer],IllocPt)
).


% achievement mode effect
drule(3,T,Agt,
    % premises = conclus of axiom 1
    ([
        % alpha-loc was done
        %%T1 is T-1,
        believe(Agt,done(Speaker,AlphaLoc,T),_DegB,T),   %%done(T1)
        % it is grounded it was done at instant T1 (ax1a)
        is_grounded([Speaker,Hearer],done(Speaker,AlphaLoc,T)),  %%T1
        % corresponding illocutionary act
        speech_act(AlphaIlloc,AlphaLoc),
        % its sincerity cond
        sincerity_cond(T,AlphaIlloc,SincCd),  %%T1
        % and its preparatory condition
        preparatory_cond(T,AlphaIlloc,PrepCd),   %%T1
        % it is grounded they were true at the previous instant
        is_grounded([Speaker,Hearer],SincCd),
        is_grounded([Speaker,Hearer],believe(Speaker,PrepCd,_Deg,T)),   %%T1
        % get its achievement mode effect now
        achiev_mode_effect(T,AlphaIlloc,AcvMdEff)
    ]),
    % conclu = achievement mode effect is true
    AcvMdEff
).


% grounded(achievement mode effect)
%% FIXME : for now deduce both cci and grounded(cci) because no demo rule allows it yet
drulec(7,T,Agt,
    % premises = conclus of axiom 1
    ([
        % alpha-loc was done
        %%T1 is T-1,
        believe(Agt,done(Speaker,AlphaLoc,T),_DegB,T),  %%done(T1)
        % it is grounded it was done at instant T (ax1a)
        is_grounded([Speaker,Hearer],done(Speaker,AlphaLoc,T)),   %%T1
        % corresponding illocutionary act
        speech_act(AlphaIlloc,AlphaLoc),
        % its sincerity cond at time of perf
        sincerity_cond(T,AlphaIlloc,SincCd),  %%T1
        % and its preparatory condition
        preparatory_cond(T,AlphaIlloc,PrepCd),  %%T1
        % it is grounded they were true at the previous instant
        is_grounded([Speaker,Hearer],SincCd),
        is_grounded([Speaker,Hearer],believe(Speaker,PrepCd,_Deg,T)),   %%T1
        % get its achievement mode effect now
        achiev_mode_effect(T,AlphaIlloc,AcvMdEff)
    ]),
    % conclu = achievement mode effect is grounded
    grounded([Speaker,Hearer],AcvMdEff)
).


%%%%%%%%
% Succesful performance
%%%%%%%%

% the definition of the performance of the illocutionary act
% is formalised as an inference rule
% if the right-part of the definition is true, then we can deduce the succesful performance of the illoc act

% commented July 10, 2013 - can be deduced form rule 31 in pleiad_grounding.pl and from conclusion of next rule here
% uncommented same day, rule 31 fails to deduce belief...
drule(4,T,Agt,
    % premises = right-part of definition
    ([
        % performance of loc act
        %%T1 is T-1,
        believe(Agt,done(Speaker,AlphaLoc,T),_DegB,T),  %% believe at T1, %%done at T1
        % corresponding illoc act
        speech_act(AlphaIlloc,AlphaLoc),
        % illoc point is true
        illoc_point(T,AlphaIlloc,IllocPt),
        IllocPt,
        % achievement mode effect is true
        achiev_mode_effect(T,AlphaIlloc,AcvMdEff),
        AcvMdEff,
        % sincerity condition: grounded it was true when performing alpha loc
        sincerity_cond(T,AlphaIlloc,SincCd),  %%T1
        is_grounded([Speaker,Hearer],SincCd),
        % preparatory condition: grounded it was believed when performing alpha loc
        preparatory_cond(T,AlphaIlloc,PrepCd),   %%T1
        is_grounded([Speaker,Hearer],believe(Speaker,PrepCd,_Deg,T))   %%T1
    ]),
    % conclusion = succesful perf of illoc act (WARNING: stimulus must be sent to both agents involved)
    % belief of observing agent, with degree 1 and source observation
    bel(Agt,done(Speaker,AlphaIlloc,T),1,T,observation)  % T1 T1
).

% deduce that perf of alpha illoc is grounded (can be proven from the defs)
% can also be deduced from new grounding rule (grounded(done) -> bel(done)) in pleiad_grounding.pl
drulec(8,T,Agt,
    % premises = right-part of definition
    ([
        % performance of loc act at the SAME time
        %T1 is T-1,
        believe(Agt,done(Speaker,AlphaLoc,T),_DegB,T),  % done at T1
        % corresponding illoc act
        speech_act(AlphaIlloc,AlphaLoc),
        % illoc point is true
        illoc_point(T,AlphaIlloc,IllocPt),
        IllocPt,
        % achievement mode effect is true
        achiev_mode_effect(T,AlphaIlloc,AcvMdEff),
        AcvMdEff,
        % sincerity condition: grounded it was true before alpha loc
        sincerity_cond(T,AlphaIlloc,SincCd),  %T1
        is_grounded([Speaker,Hearer],SincCd),
        % preparatory condition: grounded it was believed before alpha loc
        preparatory_cond(T,AlphaIlloc,PrepCd),  %T1
        is_grounded([Speaker,Hearer],believe(Speaker,PrepCd,_Deg,T))   %T1
    ]),
    % conclusion = grounded that succesful perf of illoc act
    % FIXME: Agt is not concerned? part of the group in which it is grounded?
    grounded([Speaker,Hearer],done(Speaker,AlphaIlloc,T))   %T1
).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% extra rule for performance of expressive speech act                %%
%% whatever the degree of expressed emotion, the expressive was done  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% does not work - cf pleiad_kbmgr.pl instead 
% drule(T,Agt,
%    % premise
%    ([
%        % the agent believes that some expressive was done with a particular degree
%        bel(Agt,done(Someone,express(Someone,Someother,[Emo,Object,_DegEmo]),Sometime),Deg,T,_Src)
%    ]),
%    % conclusion : the expressive can be considered done at any degree
%    % FIXME : at any lower degree only? but how to express that? cannot enumerate all lower degrees...
%    bel(Agt,done(Someone,express(Someone,Someother,[Emo,Object,_]),Sometime),Deg,T,deduction)
% ).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DIALOGUE PLANNING RULES      %%
%%   --------------------       %%
%% ie how the agent decides     %%
%% what to say next based on    %%
%% its goals, emotions, ideals, %%
%% personality...               %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% RK -- rational effect / intended effect used to set the force category
% additional conditions / relation with hearer used to set the exact force in that category

%%%%%%%%%%%%%%

%%%%
%% 1) express emotions or empathy
%%%%

% goal to express an emotion when it is felt 
prule(1,T,Agt,
    % premises
    ([
        % if the agent feels an emotion
        emotion(Agt,Emo,Prop,DegEmo,T),
        % get the expression threshold in this agent's personality TODO pleiad_perso.pl
        % and a friend is present
        ami(Agt,Ami,DegAmi),
        believe(Agt,presence(Ami),DegPres,T),
        % combine degrees
        DegIntend is DegAmi * DegEmo * DegPres,
        % no intention to express anything yet
        % not(intend(Agt,done(Agt,express(Agt,_Hearer,_EmoList),_),_,T))
        % emotion was not expressed yet in any way, ie is not grounded, even with different degree and time
        not(grounded([Agt,Ami],emotion(Agt,Emo,Prop,_DegEmo2,_T2)))
    ]),
    % conclu : new goal, source = emotion
    goal(Agt,done(Agt,express(Agt,Ami,[Emo,Prop,DegEmo]),_TT),DegIntend,T,emotion)  
).

% goal to express empathy for an emotion that is felt by someone who cares? or by a friend?
prule(2,T,Agt,
    % premises
    ([
        % if the agent believes that someone feels an emotion
        bel_other_emo(Agt,Other,Emo,Prop,DegEmo,T),
        % and this agent is a friend (FIXME: or is cared for by Agt?)
        ami(Agt,Other,DegAmi),
        % combine degrees
        DegIntend is DegAmi * DegEmo,
        % no intention to express anything yet
        not(intend(Agt,done(Agt,express(Agt,_Hearer,_EmoList),_),_,T))
    ]),
    % conclu
    goal(Agt,done(Agt,express(Agt,Other,[empathy,emotion(Other,Emo,Prop,DegEmo,T),DegIntend]),_TT),DegIntend,T,social)  
).

%%%%%%%%%%%%%%%%%%%%

%% TOO GENERIC: commented
% intention to perform a speech act that reaches an intended situation
% FIXME: how to choose interlocutor/hearer? the one that allows to fulfil the goal
% prule(T,Agt,
%      % premises
%    ([
%        % agent intends to reach a situation
%        intend(Agt,Situation,Deg,T),
%        % and there is a speech act whose illoc point is that situation
%        % FIXME: or IP = grounded(Situation) is some group of which Agt is a member?
%        speaker(SpeechAct,Agt),
%        illoc_point(T,SpeechAct,grounded(G,Situation)),
%        hearer(SpeechAct,Other),
%        member(Agt,G),
%        member(Other,G),
%        prop_content(SpeechAct,PContent),
%        ground(PContent)
%    ]),
%    % conclu = intention to perform that speech act with same degree at same time
%    intend(Agt,done(Agt,SpeechAct,_),Deg,T)
% ).


%%%%
%% 2) goal to transfer beliefs to another agent
%%%%

%% similar but intention to make another agent believe a situation
%% induces intention to ground that situation
prule(3,T,Agt,
      % premises
    ([
        % agent intends to reach a situation / goal / any ?
        goal(Agt,believe(Other,Situation,_DBel,_TBel),Deg,T,Src),
        ground(Other),Other\=Agt,
        % and there is a speech act whose illoc point is that situation
        % FIXME: or IP = grounded(Situation) is some group of which Agt is a member?
        illoc_point(T,SpeechAct,grounded(G,Situation)),
        speaker(SpeechAct,Agt),
        hearer(SpeechAct,Other),
        member(Agt,G),
        member(Other,G),
        prop_content(SpeechAct,PContent),
        % cannot impose ground content (typically queries have unground content)
        ground(PContent)
    ]),
    % conclu = intention to perform that speech act with same degree at same time
    goal(Agt,done(Agt,SpeechAct,_),Deg,T,Src)
).


%%%%
%% 3) goal to transfer intentions to another agent
%%      - goals to have an action performed
%%%%

%% goal to express/ground intentions that others do actions
prule(4,T,Agt,
      % premises
    ([
        % agent intends that other agent performs an action
        goal(Agt,done(Other,Alpha,Time),Deg,T,Src),
        ground(Other),Other\=Agt
    ]),
    % conclu = intention to perform that speech act with same degree at same time
    goal(Agt,believe(Other,intend(Agt,done(Other,Alpha,Time),Deg,T),1,T),Deg,T,Src)
).


%%%%
%% 4) goal to transfer intentions to another agent
%%      - goal to gain belief, to knowval, to knowif
%%%%

%% goal to express intention to gain a belief that the other is believed to have
prule(5,T,Agt,
      % premises
    ([
        % agent intends to learn a belief
        goal(Agt,believe(Agt,Prop,DBelA,TBelA),DInt,T,Src),
        % and agent believes that other agent has this belief
        believe(Agt,believe(Other,Prop,_DBelB,_TBelB),_DBel,T),
        ground(Other),Other\=Agt
    ]),
    % conclu = intention to perform that speech act with same degree at same time
    goal(Agt,believe(Other,intend(Agt,believe(Agt,Prop,DBelA,TBelA),DInt,T),1,T),DInt,T,Src)
).

%% goal to express intention to knowval when the other is believed to know it
prule(6,T,Agt,
      % premises
    ([
        % agent intends to know a value
        goal(Agt,knowval(Agt,Prop,TBelA),DInt,T,Src),
        % and agent believes that other agent knows this value
        believe(Agt,knowval(Other,Prop,_TBelB),_DBel,T),
        ground(Other),Other\=Agt
    ]),
    % conclu = intention to perform that speech act with same degree at same time
    goal(Agt,believe(Other,intend(Agt,knowval(Agt,Prop,TBelA),DInt,T),1,T),DInt,T,Src)
).



%% goal to express intention to knowvif when the other is believed to know if P
prule(7,T,Agt,
      % premises
    ([
        % agent intends to know if a prop is true
        goal(Agt,knowif(Agt,Prop,TBelA),DInt,T,Src),
        % and agent believes that other agent knows if it is true
        believe(Agt,knowif(Other,Prop,_TBelB),_DBel,T),
        ground(Other),Other\=Agt
    ]),
    % conclu = intention to perform that speech act with same degree at same time
    goal(Agt,believe(Other,intend(Agt,knowif(Agt,Prop,TBelA),DInt,T),1,T),DInt,T,Src)
).

% FIXME: generic rule?
% premise: intention to gain a mental attitude + belief that the other has this mental attitude
%  (mental attitude can be believe, knowif, knowval, also goals, etc)
% conclusion: intention that the other believes that the agent has this intention to gain this mental attitude
% ie rule to express beliefs about any intention that the agent has
% FIXME !



%%%%%%%%%%%%%%%%%%%%%%%%%%%%







%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DIALOGUE PROTOCOL HANDLING %%
%% in terms of grounding      %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% detect a lie as joint grounding of an intention to make the other believe phi, while believing not(phi)


% protocol : before interacting with someone, greet them
% if friend -> welcome


% protocol : answer questions ?


% protocol for politeness: provides motivation for insincere speech acts
% eg start and stop a dialogue with some greetings


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DIALOGUE MANAGEMENT (SIMPLE) %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% send a say(Child,Companion,Words) stimulus
% for child input

% interpret it in the companion


% dialogue between 2 agents
dialogue(A,B) :- turn(A),turn(B),dialogue(A,B).
dialogue(_A,_B) :- !.

% TODO : generalise the phases mechanism (perception, decision, action)

oldturnstart :-
    % current instant
    instant(T),
    % inferences in perception phase/mode
    saturer_bc(coco),
    % new JUNE 2013 : decision phase
    retract(phase(Agt,OldPhase)),
    assert(phase(Agt,decision)),
    % decide of which intention to pursue
    saturer_bc(Agt),
    % get out of decision phase - back to previous phase FIXME: or go to next phase?
    retract(phase(Agt,decision)),
    assert(phase(Agt,OldPhase)),
    % agent's intention to perform a speech act
    intend(Agt,done(Agt,_SpeechAct,_),_,T).
    

turn(Agt,Words) :-
    decide(Agt,done(Agt,SpeechAct,_),_),
    % get the corresponding words
    speech_act(SpeechAct,say(Agt,Someone,Words)),
    % perform the speech act
    % send stimulus? to both speaker and hearer
    stimulus([Agt,Someone],say(Agt,Someone,Words)).
    % retract intention? consequence of stimulus?
    % print the result for the user
    % print(Agt+':'+Words).
    
    
% whattosay(Agt,Words) :-