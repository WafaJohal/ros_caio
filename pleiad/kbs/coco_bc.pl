%%%%
% kb for aamas example of child-companion dialogue
% coco = companion agent
%%%%

:- kassert(phase(coco,perception)).

:- kassert(phase(kiki,perception)).


%%%%%%%%%%%%%%%%%%%%
%% TODO TODO      %%
%% Tuesday 2 July %%
%%%%%%%%%%%%%%%%%%%%

% pb with manage focus time not instantiated sur reactive/proactive_loop
% 


%%%%%%%%%%%%%%
%% SCENARIO %%
%%%%%%%%%%%%%%

%%%%
%% speech act 1 = welcome
%%%%
% event 1: the child enters the room =
% > stimulus([coco],event(child-enter, presence(kiki))).
% must trigger a welcome expressive speech act

% TODO neXt
% this intention should have higher priority (protocol? emotion? focus on reactive response to event?)
% than intention to ask question about activity (task)
% also when several intentions, prule should select highest priority
% and find a speech act to fulfil that one

% s1 :- perceive(coco,event(child-enter, presence(kiki))).
s1(Action1) :- agent_reactive_loop(coco,event(child-enter, presence(kiki)),Action1).
s1debug :- perceive(coco,event(child-enter, presence(kiki))),saturer_bc(coco).

% stimulus([coco],action())

%%%%
%% speech act 2 = what have you been up to
%%%%
% companion's goal to know the child activities every day
% knowledge of the current day (context)
% query ref ?

:- kassert(goal(coco,knowval(coco,activity(kiki,last),_T),1,0,task)).

% convention: knowval(Agt,Prop) is written believe(Agt,value(Prop,Prop)) to match intention
%               because should not have not instantiated value that would match everything
% pb with time of belief of other: if instantiated: the other is only believed to believe the prop at that instant
%                                   but if not instantiated: arguments of emotion predicate are not sufficiently instantiated
:- kassert(bel(coco,knowval(kiki,activity(kiki,last),_T),1,0,conviction)).   % _Val --> activity(kiki,last)
% instant(T) -> ...


s2(Action2) :- agent_proactive_loop(coco,Action2).

%%%%
%% speech act 3
%% express empathy towards the child about last activity
%%%%

%% next stimulus : kiki answers with the value of his last activity 
%% coco needs desires so that this triggers a positive emotion (empathy for the child)
%% which induces intention to express it
%% coco should know about kiki's preferences regarding that activity
%% deduce kiki's emotion and adopt intention to express empathy (be it felt or not: first insincere act)
%% because politeness / protocol says that he should express empathy towards friends when they feel an emotion

% kiki performs speech act + coco reacts to it
% warning: stimulus = locutionary act = words, and NOT illocutionary act
% the illocutionary act is the interpretation by the hearer of what was said
% inform(kiki,coco,value(activity(kiki,last),hike))
s3(Action3) :- perform(kiki,say(kiki, coco, [kiki, informs, coco, that, value(activity(kiki,last),hike)]),[]),
                agent_proactive_loop(coco,Action3).
s3debug :- perform(kiki,say(kiki, coco, [kiki, informs, coco, that, value(activity(kiki,last),hike)]),[]),
            saturer_bc(coco).

% coco believes that kiki likes hiking
:- kassert(bel(coco,like(kiki,hike,1),1,0,observation)).

% rule: agents desire that their activities are something they like
:- kassert(bel(coco,like(Agt,Activity,Deg) -> desire(Agt,value(activity(Agt,_Any),Activity),Deg),1,0,conviction)).


% now need to be aware of emotion definitions
% cf predicate bel_other_emo in pleiad_emotions.pl



%%%%
%% speech act 4
%%%%

%% the child expresses negative emotion about school
%% coco should express empathy (reactive) and then try to know more / understand
%% what triggers that intention? what does he want to know exactly?
%% does he have generic questions (insincere) just to keep the dialogue going?
%% eg tell me more, why do you say that, and so on

% spontaneous inform(kiki,coco,value(activity(kiki,next),school))
% stimulus = loc act = words
% say(kiki, coco, [kiki, informs, coco, that, value(activity(kiki,last),hike)])
s4(Action4) :- perform(kiki,say(kiki, coco, [kiki, informs, coco, that, value(activity(kiki,next),school)]),[]),agent_proactive_loop(coco,Action4).

% coco believes that kiki dislikes school
:- kassert(bel(coco,dislike(kiki,school,1),1,0,observation)).

% rule: agents desire that their activities are something they like
:- kassert(bel(coco,dislike(Agt,Activity,Deg) -> undesire(Agt,value(activity(Agt,_Any),Activity),Deg),1,0,conviction)).




%%%%
%% speech act 5 
%%%%

% kiki desires to succeed exam, undesires to fail it
:- kassert(bel(coco,desire(kiki,succeed(_),0.8),1,0,conviction)).
:- kassert(bel(coco,undesire(kiki,fail(_),0.8),1,0,conviction)).

% kid does not like maths
:- kassert(bel(coco,dislike(kiki,maths,0.9),1,0,conviction)).

% kid likes biology
:- kassert(bel(coco,like(kiki,biology,0.8),1,0,conviction)).

% probabilities to succeed or fail exam (subjective for the other depending on his liking of the topic)
% FIXME: prob(fail) ou prob(future(fail))
% :- kassert(bel(coco,like(Agt,Topic,Deg)->(future(exam(Topic))->prob(Agt,succeed(exam(Topic)),Deg,_T)),1,0,conviction)).
% :- kassert(bel(coco,dislike(Agt,Topic,Deg)->(future(exam(Topic))->prob(Agt,fail(exam(Topic)),Deg,_T)),1,0,conviction)).

% new version for transmission of focus in the network
:- kassert(bel(coco,like(Agt,Topic,Deg)->(prob(Agt,future(exam(Topic))->succeed(exam(Topic)),Deg,_T)),1,0,conviction)).
:- kassert(bel(coco,dislike(Agt,Topic,Deg)->(prob(Agt,future(exam(Topic))->fail(exam(Topic)),Deg,_T)),1,0,conviction)).


% inform(kiki,exam(maths) demain)
% pb: gestion du temps (comment informer que qqch sera vrai plus tard?)
% ou inform(oblig(done(kiki,goto-exam(maths))))

s5(Action5) :- perform(kiki,say(kiki, coco, [kiki, informs, coco, that, future(exam(maths))]),[]),
                agent_proactive_loop(coco,Action5).

s5debug :- perform(kiki,say(kiki, coco, [kiki, informs, coco, that, future(exam(maths))]),[]).


% rule: agents expect that someone (including themselves) fails an exam on a topic they dislike
% :- kassert(bel(AnyAgt,dislike(Agt,Topic,Deg) -> (happens(exam(Topic)) -> prob(AnyAgt,fail(Agt,exam(Topic)),Deg,_T)),1,0,conviction)).
% problem: only probable to fail the exam if there is an exam - how to express that exam will happen?

% and it is certainly (deg 1) undesirable to fail any exam - medium degree 0.7
% :- kassert(bel(coco,undesire(kiki,fail(kiki,exam(_AnyTopic)),0.7),1,0,conviction)).

% peut deja deduire sadness(oblig goto(exam(maths))) ?
% undesirable to be obliged to do a disliked action? to be obliged to go to a disliked proposition?


% bel(coco,dislike(kiki,maths))
% bel(coco, dislike(Matiere)->expect(fail(exam(Matiere))))
% undesire(fail(Anything))
% deduce fear(kiki,fail(exam(maths)))
% express empathy(fear)

%%%%
%% speech act 6
%%%%

% regle de politesse (rajouter politesse comme source de goal? ou = social?)
% rassurer un ami qui a peur
% promise(will not fail)
% promesse non sincere car aucune info ne permet d'assurer que kiki va reussir

%%%%
%% speech act 7
%%%%

% inform(kiki,failed(exam(maths)))
% sorryfor(coco,kiki)

%%%%
%% speech act 8
%%%%

% blame / reproach(kiki,coco) pour avoir dit qu'il reussirait alors qu'il a rate
% resp(coco,say(P) while not(P)) -> sorry
% apologise(coco,kiki,mistaken)


%%%%%%%%%%%%%%%%%%%%%
% kb companion coco %
%%%%%%%%%%%%%%%%%%%%%


%%%%
% desires and goals
%%%%

:- kassert(desire(coco,presence(kiki),1)).



%%%%%%
%% FRIENDS
%%%%%%

:- kassert(ami(coco,kiki,1)).


%%%%%%%%%%%
%% TRUST and IMAGE
%%%%%%%%%%%

% image that coco has of kiki
:- kassert(bel(coco,sincere(kiki),0.75,0,conviction)).
:- kassert(bel(coco,reliable(kiki),0.77,0,conviction)).
:- kassert(bel(coco,competent(kiki),0.6,0,conviction)).
:- kassert(bel(coco,care(kiki),0.9,0,conviction)).


% image that coco believes that kiki has of coco
:- kassert(bel(coco,believe(kiki,sincere(coco)  ,1,0),1,0,conviction)).
:- kassert(bel(coco,believe(kiki,reliable(coco) ,1,0),1,0,conviction)).
:- kassert(bel(coco,believe(kiki,competent(coco),1,0),1,0,conviction)).
:- kassert(bel(coco,believe(kiki,care(coco)     ,1,0),1,0,conviction)).


%%%%%%%%%%%%%%
%% COMMENTS %%
%%%%%%%%%%%%%%

% add personality-based rule to express felt emotions, based on type of relationship with interlocutor
% ie closeness to them, trust, etc

% other more "manipulative" rules (for insincere agent) might make it express emotions it does not have (eg empathy)
% could be used to compare a "sincere empathetic" agent with an "insincere empathetic" agent
% both express empathy, but one feels it while the other does not... how can we see the difference?

