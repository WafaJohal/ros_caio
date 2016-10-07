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

% tag for debug: stim

%%%%%%%%%%%%%%%%%%
%% HOW IT WORKS %%
%%%%%%%%%%%%%%%%%%

% stimulus handles single agents or groups, and calls stimulus_group (retro-compat)
% stimulus_group calls stimulus_agent for each agent in the group (dispatch)
% stimulus_agent calls perceive = the perception function of the agent
% perceive calls perceive_stimulus that handles the different types of stimuli
% perceive can also be called from the outside to make one agent perceive one stimulus

% new JULY2013: avancer is not called after stimulus anymore, but after act() in pleiad_reasoning.pl

%%%%%%%%%%%%%%%%%%%%%%
%% NOTES ---- NOTES %%
%%%%%%%%%%%%%%%%%%%%%%

%% TODO October 2012
%% manage with perception.pl that translates events in the physical world
% into stimuli for the agent (if they are above attention threshold)
%% only stimuli in focused topics should be perceived? (eg denial strategy)
%% or the salience of a stimuli out of focus should be higher for it to be perceived 

%% TODO November 2012
%% relevant() predicate to tell what a stimulus is relevant to
%% different for actions and events
%% for now any stimulus gets the focus
%% but the previous focus should actually influence if the stimulus is perceived or not (awareness)

%% NEW APRIL 2013 = group stimuli
% 2) TODO later stimuli could take a 3rd parameter = the group of witnesses
% or take a list of agents instead of one agent alone as 1st param


% PB APRIL 2013
% with the different modules, some stimuli are not defined if some files are not compiled
% in particular speech acts with illoc force and prop content are only defined in pleiad_language_jeremy or pleiad_speechacts_benoit
% TODO: find a way to make compiling all modules independant, eg have a default language module compiled every time

% NEW JUNE 2013 (cf svg before change in drafts) :
% stimulus is now called perceive and is a predicate for one agent.
% new stimulus predicate sends a stimulus to a group of agents, and triggers perception function of each agent
% also time advances after any type of stimulus
% 1) stimulus(Group,Stimulus) sends stimulus to all agents in the group
% 2) stimulus_


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% discontiguous predicates %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

:- discontiguous(stimulus/2).
:- discontiguous(stimulus_agent/2).
:- discontiguous(stimulus_group/2).


%%%%%%%%%%%%
%% debug  %%
%%%%%%%%%%%%

debug_stimulus :-
    trace(stimulus),trace(stimulus_group), trace(stimulus_agent),
    trace(perceive_stimulus),trace(perceive),trace(assert_belief).
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                            %%
%%         1) STIMULUS        %%
%%                            %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% GENERIC STIMULUS PREDICATE         %%
%% retro-compatibility                %%
%% deals with groups or single agents %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% in each case: move time forward
% even if no agent receives the stimulus or the agent does not understand it
% new July 2013 : time is not moved forward inside the call to stimulus (because problems of KB not being updated in time)

% non-empty group: send stimulus to group
stimulus(Group,Stimulus) :- 
		% Group is a non-empty list
		is_list(Group),member(_,Group),
		% use the group-stimulus predicate
		stimulus_group(Group,Stimulus),!.  %,avancer.

% empty group: do nothing
stimulus([],_Stimulus) :- !. %,avancer.

% single agent: retro-compatibility
stimulus(Agent,Stimulus) :-
	not(is_list(Agent)),
        stimulus_agent(Agent,Stimulus),!. %,avancer.

%% default stimulus (not an event, action, speech act, say) : error message
% the other types of stimuli are not interpreted, sends a message to the user
stimulus(A,E) :- display_message(['agent ',A,' does not understand stimulus ',E,' ']),!. %,avancer.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% stimulus to a group      %%
%% dispatch to all agents   %%
%% advance time at the end  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% new June 2013: always advance time, even after events (not only actions as before)
% so no need to differentiate type of stimulus at this point anymore
stimulus_group([A],Stimulus) :- stimulus_agent(A,Stimulus),!.
stimulus_group([A|G],Stimulus) :- stimulus_agent(A,Stimulus),stimulus_group(G,Stimulus),!.



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% stimulus to one agent                        %%
%% triggers perception function of this agent   %%
%% that will deal with its focus, etc           %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

stimulus_agent(A,Stimulus) :-
    perceive(A,Stimulus).
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                                %%
%%         2) PERCEPTION          %%
%%                                %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% perception / interpretation of stimuli %%%%%%%
%%%%%%% BY ONE single agent                    %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 1) all stimuli put their object into focus -> a stimulus attracts attention
% last stimulus is remembered for each agent


% for artificial agents
% stim = event(E,P) ou action(Resp,Alpha,Csq) ou SpeechAct
perceive(Agt,Stim) :-
    % check a KB file exists for this agent (cf pleiad_tools.pl) or it is the current agent
    % WARNING : the KB file should have the same name as the agent that owns it! because artificial_agent checks filenames
    (current_agent(Agt) | artificial_agent(Agt)),
    % change phase
    % ((retract(phase(Agt,action)), assert(phase(Agt,perception)),!);phase(Agt,perception)),
    enter_perception_phase(Agt),
    % perceive stimulus (advances time)
    % TODO : apply possible filters depending on agent focus
    perceive_stimulus(Agt,Stim),
    % saturate KB
    saturer_bc(Agt).

% for human agents : do nothing
perceive(Agt,_Stim) :-
    human_agent(Agt),!.


%%%%%%%%%%%%%
%% EVENT STIMULUS
%%%%%%%%%%%%%

% agent A receives as stimulus an event named E whose consequences are described by P
% TODO later: use a list of consequences instead of a single P and handle all consequences in the list
% TODO later: list of witnesses of the event?
perceive_stimulus(A,event(E,P)) :- 
		% move time forward - only for actions (can be several events at each instant)
		% avancer,   
		% checks and instantiations
		ground(E),ground(P),instant(T),   
		% effects of stimulus + auto focus
		assert_new_belief(A,P,1,T,observation),
                nofail(retract(laststimulus(A,_))),
		assert(laststimulus(A,event(E,P))),
		!.
                

%%%%%%%%%%
%% ACTION STIMULUS
%%%%%%%%%%

% A receives as stimulus the action named Alpha, performed by an agent Resp, with consequences Csq
perceive_stimulus(A,action(Resp,Alpha,Csq)) :-
		% checks instantiation
		ground(Alpha),ground(Resp),ground(Csq),instant(T),
		% belief : memory (done alpha)
		assert_belief(A,action(Resp,Alpha,T),1,T,observation),            
		% belief about consequences
		assert_new_belief(A,Csq,1,T,observation),                         
		% belief on who is responsible for the action
		assert_belief(A,resp(Resp,Alpha,Csq),1,T,observation),            
		% focus on the action
		set_focus(A,Alpha,1,T),
		% memory of last stimulus = that action
                nofail(retract(laststimulus(A,_))),
		assert(laststimulus(A,action(Resp,Alpha,Csq))),
		!.
                % avancer in the end, removed to be done only once the whole group received the stimulus
        


%%%%%%%%%%%%%%
%% SPEECH ACT STIMULUS (Jeremy's language)
%%%%%%%%%%%%%%

% NEW from Jeremy dialogue_event.pl
% Speech Act stimulus					
perceive_stimulus(A,SpeechAct) :-
	% Speech Act decomposition
	illoc_force(SpeechAct,FI),
	hearer(SpeechAct,Hearer),
	speaker(SpeechAct,Speaker),
	prop_content(SpeechAct,CP),
	ground(FI),ground(Speaker),ground(Hearer),instant(T),   
	assert_belief(A,done(Speaker,SpeechAct,T),1,T,observation),
	nofail(retract(laststimulus(A,_))),
	assert(laststimulus(A,SpeechAct)),
	%T1 is T + 1,
	set_focus(A,CP,1,T),			
	%avancer,
        !.        
       
% new July2014 - splash language
% splash(A,Speaker,Hearer,Force,Sujet,Verbe,Complement)
% :- speaker(SA,Speaker),hearer(SA,Hearer),illoc_force(SA,Force),prop_content(SA,splash(Sujet,Verbe,Complement)),
%    perceive_stimulus(A,SA).
        
              
%%%%%%%%%%%%%%%%
%% LOCUTIONARY ACT STIMULUS (Benoit's language)
%% TODO june2014 - to be corrected with newformat speechact(force,speaker,hearer,cprop)
%%                 and also correct file semantics_benoit.pl
%%%%%%%%%%%%%%%%
                                
% new stimulus = utter words
% working with pleiad_speechacts_benoit.pl
perceive_stimulus(A,say(Speaker,Hearer,Words)) :-
    instant(T),
    % corresponding illocutionary act
    speech_act(IllocAct,say(Speaker,Hearer,Words)),
    % decompose it
    prop_content(IllocAct,CP),
    % send the illoc-act stimulus instead? stimulus(A,IllocAct) ? no because just an attempt
    % speaker did perform the locutionary act ?
    assert_belief(A,done(Speaker,say(Speaker,Hearer,Words),T),1,T,observation),
    % update last stimulus and focus
    nofail(retract(laststimulus(A,_))),
    assert(laststimulus(A,say(Speaker,Hearer,Words))),
    % set focus at THIS instant
    %T1 is T+1,
    % focus can only be set on an atom
    set_focus(A,CP,1,T),
    % assert A's belief that speaker intends to do the alpha-illoc corresp ?
    % advance to next instant
    %%avancer,
    !.
    
% new stimulus = utter words
% error case = unknown words (not matching an illoc act in grammar)
perceive_stimulus(A,say(_Speaker,_Hearer,Words)) :-
    print('agent '+A+' received stimulus utterance of words '+Words+' that are not in grammar...'),!.





    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                              %%
%%         3) AUXILIARY         %%
%%                              %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    

%%%%%%%%%%%%%
%%%% auxiliary functions
%%%%%%%%%%%%%

%% Rk October 2012
%% these are unused..

%% a prop is interesting if the agent has desires/undesires about it
p_interest(A,P) :- desire(A,P,_). 
p_interest(A,P) :- desire(A,not(P),_). 
p_interest(A,P) :- undesire(A,P,_).
p_interest(A,P) :- undesire(A,not(P),_).

%% an action is interesting if the agent has norms or expectations about it
a_interest(A,Alpha,T) :- believe(A,ideal(_,Alpha,_),_,T). 
a_interest(A,Alpha,T) :- believe(A,unideal(_,Alpha,_),_,T). 
a_interest(A,Alpha,T) :- believe(A,ideal(_,not(Alpha),_),_,T).
a_interest(A,Alpha,T) :- believe(A,unideal(_,not(Alpha),_),_,T). 
a_interest(A,Alpha,T) :- believe(A,unexpected(_,Alpha,_),_,T). 
a_interest(A,Alpha,T) :- believe(A,unexpected(_,not(Alpha),_),_,T).
a_interest(A,Alpha,T) :- believe(A,expected(_,Alpha,_),_,T). 
a_interest(A,Alpha,T) :- believe(A,expected(_,not(Alpha),_),_,T).



