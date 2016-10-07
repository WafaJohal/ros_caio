%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                                                       %%
%% PLEIAD : ProLog Emotional Intelligent Agents Designer %%
%%                                                       %%
%% Author : Carole Adam                                  %%
%% Version : v7.7 - May 2013                             %%
%%                                                       %%
%%    ------- DO NOT DISTRIBUTE -------                  %%
%%                                                       %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% grammar and semantics of speech acts
% grammar = correspondance between locutionary (words) and illocutionary act
% semantics = illocutionary point, sincerity conditions, etc of each category of speech acts
% compiled from speechacts_benoit.pl

% Modif june2014 - added src of intention (never instantiated here)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% GRAMMAR                             %%
%% correspondance between speech acts, %%
%% locutionary and illocutionary acts  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% speech_act(IllocAct,LocAct)
% where LocAct = say(Speaker,Hearer,Words)
% and IllocAct = Force(Speaker,Hearer,CProp)
% and Words is a list of words to say, in order to try to perform the speech act

%%%%%%
% ASSERTIVES
%%%%%%

% assert
speech_act(assert(Speaker,Hearer,CProp),say(Speaker,Hearer,[Speaker,asserts,to,Hearer,that,CProp])).

hearer(assert(_A,B,_P),B) :- !.
speaker(assert(A,_B,_P),A) :- !.
illoc_force(assert(_A,_B,_P),assert) :- !.
prop_content(assert(_A,_B,P),P) :- !.

% inform
speech_act(inform(Speaker,Hearer,CProp),say(Speaker,Hearer,[Speaker,informs,Hearer,that,CProp])).

hearer(inform(_A,B,_P),B) :- !.
speaker(inform(A,_B,_P),A) :- !.
illoc_force(inform(_A,_B,_P),inform) :- !.
prop_content(inform(_A,_B,P),P) :- !.

%%%%%%%
% COMMISSIVES
%%%%%%%

% commit
speech_act(commit(Speaker,Hearer,done(Speaker,Alpha,_T)),say(Speaker,Hearer,[Speaker,commits,towards,Hearer,to,perform,action,Alpha])).

hearer(commit(_A,B,_P),B) :- !.
speaker(commit(A,_B,_P),A) :- !.
illoc_force(commit(_A,_B,_P),commit) :- !.
prop_content(commit(_A,_B,P),P) :- !.

% promises
speech_act(promise(Speaker,Hearer,done(Speaker,Alpha,_T)),say(Speaker,Hearer,[Speaker,promises,to,Hearer,to,perform,action,Alpha])).

hearer(promise(_A,B,_P),B) :- !.
speaker(promise(A,_B,_P),A) :- !.
illoc_force(promise(_A,_B,_P),promise) :- !.
prop_content(promise(_A,_B,P),P) :- !.

%%%%%%%
% DIRECTIVES
%%%%%%%

% direct
speech_act(direct(Speaker,Hearer,done(Hearer,Alpha,_T)),say(Speaker,Hearer,[Speaker,directs,Hearer,to,perform,action,Alpha])).

hearer(direct(_A,B,_P),B) :- !.
speaker(direct(A,_B,_P),A) :- !.
illoc_force(direct(_A,_B,_P),direct) :- !.
prop_content(direct(_A,_B,P),P) :- !.

% request
speech_act(request(Speaker,Hearer,done(Hearer,Alpha,_T)),say(Speaker,Hearer,[Speaker,requests,Hearer,to,perform,action,Alpha])).

hearer(request(_A,B,_P),B) :- !.
speaker(request(A,_B,_P),A) :- !.
illoc_force(request(_A,_B,_P),request) :- !.
prop_content(request(_A,_B,P),P) :- !.

% tell
speech_act(tell(Speaker,Hearer,done(Hearer,Alpha,_T)),say(Speaker,Hearer,[Speaker,tells,Hearer,to,perform,action,Alpha])).

hearer(tell(_A,B,_P),B) :- !.
speaker(tell(A,_B,_P),A) :- !.
illoc_force(tell(_A,_B,_P),tell) :- !.
prop_content(tell(_A,_B,P),P) :- !.


% queryif
speech_act(queryif(Speaker,Hearer,Prop),say(Speaker,Hearer,[Speaker,queries,Hearer,if,Prop])).

hearer(queryif(_A,B,_P),B) :- !.
speaker(queryif(A,_B,_P),A) :- !.
illoc_force(queryif(_A,_B,_P),queryif) :- !.
prop_content(queryif(_A,_B,P),P) :- !.


% queryval
speech_act(queryval(Speaker,Hearer,Prop),say(Speaker,Hearer,[Speaker,queries,Hearer,value,of,Prop])).

hearer(queryval(_A,B,_P),B) :- !.
speaker(queryval(A,_B,_P),A) :- !.
illoc_force(queryval(_A,_B,_P),queryval) :- !.
prop_content(queryval(_A,_B,P),P) :- !.



%%%%%%%
% EXPRESSIVES
%%%%%%%

% specific expressives

% welcome
speech_act(express(Speaker,Hearer,[joy,presence(Hearer),_Deg]),say(Speaker,Hearer,[welcome,Hearer,i,am,glad,to,see,you])).

% generic express
% the info in the prop content actually defines more specific expressive acts...
speech_act(express(Speaker,Hearer,[Emo,Object,Deg]),say(Speaker,Hearer,[Speaker,expresses,Emo,about,Object,at,degree,Deg,to,agent,Hearer])).

hearer(express(_A,B,_P),B) :- !.
speaker(express(A,_B,_P),A) :- !.
illoc_force(express(_A,_B,_P),express) :- !.
prop_content(express(_A,_B,P),P) :- !.



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% SPECIFIC CONDITIONS       %%
%% AND EFFECTS               %%
%% OF SPEECH ACTS CATEGORIES %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% conditions for all speech acts
% cond_spact(Agt,SpeechAct) :-

%%%%%%
% ASSERTIVES
%%%%%%

% T is a parameter: check the conditions or effects at a given explicit instant

% inform
illoc_point(T,inform(Speaker,Hearer,Cprop),
            grounded([Speaker,Hearer],believe(Speaker,Cprop,1,T))).
% and grounded([Speaker,Hearer],intend(Speaker,believe(Hearer,Cprop,_D,T),1,T,_S) ???
achiev_mode_effect(_T,inform(_Speaker,_Hearer,_Cprop),
            true).
sincerity_cond(T,inform(Speaker,_Hearer,Cprop),
            believe(Speaker,Cprop,1,T)).
% and intend ??? no, wouldn't perform this speech act if didn't have this intention
preparatory_cond(T,inform(_Speaker,Hearer,Cprop),
            not(believe(Hearer,Cprop,_Deg,T))).

% assert
illoc_point(T,assert(Speaker,Hearer,Cprop),
            grounded([Speaker,Hearer],believe(Speaker,Cprop,1,T))).
achiev_mode_effect(_T,assert(_Speaker,_Hearer,_Cprop),
            true).
preparatory_cond(_T,assert(_Speaker,_Hearer,_Cprop),
            true).
sincerity_cond(T,assert(T,Speaker,_Hearer,Cprop),
            believe(Speaker,Cprop,1,T)).



%%%%%%%
% COMMISSIVES
%%%%%%%

% commit
illoc_point(T,commit(Speaker,Hearer,done(Speaker,Action,Sometime)),
        grounded([Speaker,Hearer],intend(Speaker,done(Speaker,Action,Sometime),1,T,_Src))).
achiev_mode_effect(_T,commit(Speaker,_Hearer,done(Speaker,_Action,_Sometime)),
        true).
% first prep cond: speaker does not believe he is not capable to perform the action
preparatory_cond(T,commit(Speaker,_Hearer,done(Speaker,Action,_Sometime)),
        not(believe(Speaker,not(capable(Speaker,Action)),_Deg,T))).
% second preparatory condition: hearer does not believe either that speaker is incapable of the action
preparatory_cond(T,commit(Speaker,Hearer,done(Speaker,Action,_Sometime)),
        not(believe(Hearer,not(capable(Speaker,Action)),_Deg,T))).
% sincerity cond = intention to perform action
sincerity_cond(T,commit(Speaker,_Hearer,done(Speaker,Action,Sometime)),
        intend(Speaker,done(Speaker,Action,Sometime),1,T,_Src)).

% promise
illoc_point(T,promise(Speaker,Hearer,done(Speaker,Action,Sometime)),
        grounded([Speaker,Hearer],intend(Speaker,done(Speaker,Action,Sometime),1,T,_Src))).
achiev_mode_effect(T,promise(Speaker,Hearer,done(Speaker,Action,Sometime)),
        %% FIXME: time for obligation / time for performance : the same ? T ? T1 ?
        grounded([Speaker,Hearer],believe(Speaker,oblig(done(Speaker,Action,Sometime)),1,T))).  % :- T1 is T+1.
% prep cond: the hearer has a goal that the speaker performs the promised action (positive action for the hearer of the promise)
preparatory_cond(_T,promise(Speaker,Hearer,done(Speaker,Action,Sometime)),
        goal(Hearer,done(Speaker,Action,Sometime),_SomeDeg,Sometime,_Src)).
% sinc cond: the speaker actually intends to perform the promised action
sincerity_cond(T,promise(Speaker,_Hearer,done(Speaker,Action,Sometime)),
        intend(Speaker,done(Speaker,Action,Sometime),1,T,_Src)).


%%%%%%%
% DIRECTIVES
%%%%%%%

% direct
illoc_point(T,direct(Speaker,Hearer,done(Hearer,Action,Sometime)),
        grounded([Speaker,Hearer],intend(Speaker,done(Hearer,Action,Sometime),1,T,_Src))).
achiev_mode_effect(_T,direct(_Speaker,Hearer,done(Hearer,_Action,_Sometime)),
        true).
preparatory_cond(T,direct(Speaker,Hearer,done(Hearer,Action,_Sometime)),
        not(believe(Speaker,not(capable(Hearer,Action)),_Deg,T))).
preparatory_cond(T,direct(_Speaker,Hearer,done(Hearer,Action,_Sometime)),
        not(believe(Hearer,not(capable(Hearer,Action)),_Deg,T))).
sincerity_cond(T,direct(Speaker,Hearer,done(Hearer,Action,Sometime)),
        intend(Speaker,done(Hearer,Action,Sometime),1,T,_Src)).

% request
illoc_point(T,request(Speaker,Hearer,done(Hearer,Action,Sometime)),
        grounded([Speaker,Hearer],intend(Speaker,done(Hearer,Action,Sometime),1,T,_Src))).
achiev_mode_effect(T,request(Speaker,Hearer,done(Hearer,Action,_Sometime)),
        % option de refus = g-k not oblig not g-k intend-j not done-j-alpha top
        % FIXME: utilite de grounder une non-obligation?
        grounded([Speaker,Hearer],not(oblig(not(grounded([Speaker,Hearer],intend(Hearer,not(done(Hearer,Action,_)),1,T,_Src))))))
        ).
preparatory_cond(T,request(Speaker,Hearer,done(Hearer,Action,_Sometime)),
        not(believe(Speaker,not(capable(Speaker,Action)),_Deg,T))).
preparatory_cond(T,request(Speaker,Hearer,done(Hearer,Action,_Sometime)),
        not(believe(Hearer,not(capable(Speaker,Action)),_Deg,T))).
sincerity_cond(T,request(Speaker,Hearer,done(Hearer,Action,Sometime)),
        intend(Speaker,done(Hearer,Action,Sometime),1,T,_Src)).

% tell
illoc_point(T,tell(Speaker,Hearer,done(Hearer,Action,Sometime)),
        grounded([Speaker,Hearer],intend(Speaker,done(Hearer,Action,Sometime),1,T,_Src))).
achiev_mode_effect(_T,tell(Speaker,Hearer,done(Hearer,Action,_Sometime)),
        % pas d'option de refus = g-k oblig not g-k intend-j not done-j-alpha top
        grounded([Speaker,Hearer],oblig(not(grounded([Speaker,Hearer],intend(Hearer,not(done(Hearer,Action,_T2)),_D3,_T3,_Src3)))))
        ).
preparatory_cond(T,tell(Speaker,Hearer,done(Hearer,Action,_Sometime)),
        not(believe(Speaker,not(capable(Speaker,Action)),_Deg,T))).
preparatory_cond(T,tell(Speaker,Hearer,done(Hearer,Action,_Sometime)),
        not(believe(Hearer,not(capable(Speaker,Action)),_Deg,T))).
sincerity_cond(T,tell(Speaker,Hearer,done(Hearer,Action,Sometime)),
        intend(Speaker,done(Hearer,Action,Sometime),1,T,_Src)).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% old version of query, replaced with queryif using new knowif abbreviation
%% query : not an intention that an action is done, but an intention to gain knowledge
%%         (and cannot say which action A intends B to do to give him this knowledge)
%illoc_point(T,queryif(Speaker,Hearer,Prop),
%        grounded([Speaker,Hearer],intend(Speaker,believe(Speaker,Prop,_Deg,_Time),1,T,_Src))).
%achiev_mode_effect(_T,queryif(_Speaker,_Hearer,_Prop),
%        true).
%% prep cond: hearer is not believed to be unable to answer (ie not believed to ignore the answer)
%% RK --- actually the planning rule gives intention to express this intention to know,
%%        only to an agent who is believed to know, so the prep cond would be valid
%preparatory_cond(T,queryif(Speaker,Hearer,Prop),
%        not(believe(Speaker,not(believe(Hearer,Prop,_Deg,_Time)),_Deg2,T))).
%% sinc cond: the speaker really has the expressed intention (to believe Prop)
%% RK --- and this intention is why he selects this speech act in the first place
%sincerity_cond(T,queryif(Speaker,_Hearer,Prop),
%        intend(Speaker,believe(Speaker,Prop,_Somedeg,_Sometime),_Deg,T,_Src)).%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% queryif : not an intention that an action is done, but an intention to gain knowledge
%         (and cannot say which action A intends B to do to give him this knowledge: inform, assert, or anything)
illoc_point(T,queryif(Speaker,Hearer,Prop),
        grounded([Speaker,Hearer],intend(Speaker,knowif(Speaker,Prop,_Time),1,T,_Src))).
achiev_mode_effect(_T,queryif(_Speaker,_Hearer,_Prop),
        true).
% prep cond: hearer is not believed to be unable to answer (ie not believed to ignore the answer)
% RK --- actually the planning rule gives intention to express this intention to know,
%        only to an agent who is believed to know, so the prep cond would be valid
preparatory_cond(T,queryif(Speaker,Hearer,Prop),
        not(believe(Speaker,not(knowif(Hearer,Prop,_Time)),_Deg2,T))).
% sinc cond: the speaker really has the expressed intention (to know IF Prop is true)
% RK --- and this intention is why he selects this speech act in the first place
sincerity_cond(T,queryif(Speaker,_Hearer,Prop),
        intend(Speaker,knowif(Speaker,Prop,_Sometime),1,T,_Src)).

        

% queryval : know the value of a property (not true-false)
illoc_point(T,queryval(Speaker,Hearer,Prop),
        grounded([Speaker,Hearer],intend(Speaker,knowval(Speaker,Prop,_Time),1,T,_Src))).
        % intend(Speaker,believe(Speaker,value(Prop,_Val),_Deg,_Time,_Src)...
achiev_mode_effect(_T,queryval(_Speaker,_Hearer,_Prop),
        true).
% prep cond: hearer is not believed to be unable to answer (ie not believed to ignore the answer)
preparatory_cond(T,queryval(Speaker,Hearer,Prop),
        not(believe(Speaker,not(knowval(Hearer,Prop,T)),_Deg,T))).
% sinc cond: the speaker really has the expressed intention (to know the value of Prop)
sincerity_cond(T,queryval(Speaker,_Hearer,Prop),
        intend(Speaker,knowval(Speaker,Prop,_Sometime),1,T,_Src)).

        


%%%%%%%
% EXPRESSIVES
%%%%%%%

% one generic expressive act = express, cprop = emotion with object and degree

%% HERE HERE HERE ONGOING APRIL 27 2013 !!

% illoc point = intention to ground the emotion? not necessarily, could just be venting...
% but if the illoc point = true, why would the agent decide to select this act?
illoc_point(_T,express(_Speaker,_Hearer,[_Emo,_Object,_Deg]),
            true).
            % grounded([Speaker,Hearer],emotion(Speaker,Emo,Object,Deg,T))).
% achievement mode is true
achiev_mode_effect(_T,express(_Speaker,_Hearer,[_Emo,_Object,_Deg]),
            true).
% sincerity condition: not a belief, but an emotion that is felt by the speaker
sincerity_cond(T,express(Speaker,_Hearer,[Emo,Object,Deg]),
            emotion(Speaker,Emo,Object,Deg,T)).
% can express the same emotion several times: no prep cond that hearer does not know
preparatory_cond(_T,express(_Speaker,_Hearer,[_Emo,_Object,_Deg]),
            true).
            % not(believe(Hearer,Cprop,_Deg,T))).


