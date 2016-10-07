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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% writing in a file           %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% open file
kao_open(F,Option,Str) :- ifthenelse(exists_file(F),(delete_file(F),open(F,Option,Str)),open(F,Option,Str)).

% delete file
kao_delete(F) :- ifthenelse(exists_file(F),delete_file(F),true).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% functions used in Delphi interface    %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% TODO October 2012 : to be used in Java GUI instead of Delphi

%%%%%%%
%% search beliefs by decreasing focus
%%%%%%%

% write beliefs vector line by line in a file
write_beliefs(A,T,F) :- vecteur_belief(A,T,L),
					kao_open(F,write,Str),
					forall(member(B,L),
						(write(Str,B),
						write(Str,'\n'))),
					close(Str).

write_purebel(A,T,F) :- vecteur_purebel(A,T,L),
					kao_open(F,write,Str),
					forall(member(B,L),(write(Str,B),write(Str,'\n'))),
					close(Str).

write_ideal(A,T,F) :- vecteur_ideal(A,T,L),
					kao_open(F,write,Str),
					forall(member(B,L),(write(Str,B),write(Str,'\n'))),
					close(Str).

write_expect(A,T,F) :- vecteur_expect(A,T,L),
					kao_open(F,write,Str),
					forall(member(B,L),(write(Str,B),write(Str,'\n'))),
					close(Str).

%%%%%%%
%% search desires sorted by decreasing degree
%%%%%%%

% write the desires vector line by line in a file
write_desires(A,F) :- vecteur_desire(A,L),
			kao_open(F,write,Str),
			forall(member(B,L),(write(Str,B),write(Str,'\n'))),
                	close(Str).

%%%%%%%
% write the agent emotion in a file
%%%%%%%

write_emotion(Agt,F) :- instant(T),
					%%emo_intens(Agt,E,T),
					express_emotion(Agt,E,T),
					kao_open(F,write,Str),
					write(Str,E),
					close(Str),!.

write_emotion(_,F) :-    kao_open(F,write,Str),
					write(Str,'neutral'),
					close(Str).

%%%%%%%
% write all emotions in a file
%%%%%%%

write_all_emotions(A,T,F) :- vecteur_emo(A,T,V),sort_emo(V,VS),
						kao_open(F,write,Str),
						forall(member(Emo,VS),(write(Str,Emo),write(Str,'\n'))),
						close(Str).


%%%%%%%
% write current instant in a file
%%%%%%%

write_time(F) :- instant(T),
				kao_open(F,write,Str),
				write(Str,T),
				close(Str).


%%%%%%%
% write all focus in a file
%%%%%%%

write_focus(A,T,F) :- kao_open(F,write,Str),
					forall((focus(A,Prop,Deg,T),Deg>0),(write(Str,focus(A,Prop,Deg,T)),write(Str,'\n'))),
					close(Str).


%%%%%%%%
%% write all relationships (friends and enemies)
%%%%%%%%

write_amis(A,F) :- vecteur_amis(A,L),
				kao_open(F,write,Str),
				forall(member(Ami,L),(write(Str,Ami),write(Str,'\n'))),
				close(Str).
				

%%%%%%%%%%%
%% save all content of an agent's KB
%% in appropriate KB file
%%%%%%%%%%%

save_time(A) :- instant(T),
				%%saturer_bc,
				saturer_bc(A),  % full saturation for THIS AGENT
				file_name(A,T,F),
				kao_open(F,write,Str),
				%% beliefs, expect, ideal
				forall(bel(A,Prop,Deg,Time,Src),write_assert(Str,bel(A,Prop,Deg,Time,Src))),
				%% desires undesires
				forall(desire(A,Prop,Deg),write_assert(Str,desire(A,Prop,Deg))),
				forall(undesire(A,Prop,Deg),write_assert(Str,undesire(A,Prop,Deg))),
				%% focus
				forall(focus(A,Prop,Deg,Time),write_assert(Str,focus(A,Prop,Deg,Time))),
				%% last stimulus
				forall(laststimulus(A,S),write_assert(Str,laststimulus(A,S))),
				%% only save emotions at times T1<T
				forall((emotion(A,Emo,Prop,Deg,Time),Time<T),write_assert(Str,emotion(A,Emo,Prop,Deg,Time))),
				%% friends and enemies relationships
				forall(ami(A,B,D),write_assert(Str,ami(A,B,D))),
				forall(ennemi(A,B,D),write_assert(Str,ennemi(A,B,D))),
				%% current instant to replace instant 0 that was asserted by compiling pleiad.pl
				write(Str,':- retract(instant(0)).\n'),
				write_assert(Str,instant(T)),
				%% close
				close(Str).

%%% save time with name of file to be specified
save_time(A,F) :- instant(T),
				%%saturer_bc,
				saturer_bc(A),  % full saturation for THIS AGENT
				%file_name(A,T,F),
				kao_open(F,write,Str),
				%% beliefs, expect, ideal
				forall(bel(A,Prop,Deg,Time,Src),write_assert(Str,bel(A,Prop,Deg,Time,Src))),
				%% desires undesires
				forall(desire(A,Prop,Deg),write_assert(Str,desire(A,Prop,Deg))),
				forall(undesire(A,Prop,Deg),write_assert(Str,undesire(A,Prop,Deg))),
				%% focus
				forall(focus(A,Prop,Deg,Time),write_assert(Str,focus(A,Prop,Deg,Time))),
				%% last stimulus
				forall(laststimulus(A,S),write_assert(Str,laststimulus(A,S))),
				%% only save emotions at times T1<T
				forall((emotion(A,Emo,Prop,Deg,Time),Time<T),write_assert(Str,emotion(A,Emo,Prop,Deg,Time))),
				%% friends and enemies relationships
				forall(ami(A,B,D),write_assert(Str,ami(A,B,D))),
				forall(ennemi(A,B,D),write_assert(Str,ennemi(A,B,D))),
				%% current instant to replace instant 0 that was asserted by compiling pleiad.pl
				write(Str,':- retract(instant(0)).\n'),
				write_assert(Str,instant(T)),
				%% close
				close(Str).


%% auxiliary function to write a prolog line in a file

write_assert(Str,Pred) :- write(Str,':- '),
						write(Str,'kassert('),
						write(Str,Pred),
						write(Str,').\n').

%% auxiliary function to give the KB file name for a given agent

file_name(A,T,F) :- string_concat('kbs/',A,F1),
				string_concat(F1,'_bc',F2),
				string_concat(F2,T,F3),
				string_concat(F3,'.pl',F).


%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% list of loaded agents %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% initially empty : asserted in main file
% list_loaded_agents([]) :- not(list_loaded_agents(_)).

% predicate to add an agent
add_loaded_agent(Agt) :- 
	list_loaded_agents(OldList),
	% if Agt is not already loaded
	not(member(Agt,OldList)),
	% append it to the list
	append([Agt],OldList,NewList),
	% retract old list
	retract(list_loaded_agents(OldList)),
	% assert new list instead
	assert(list_loaded_agents(NewList)),!.

% if agent was already loaded: inform user
add_loaded_agent(Agt) :-
	list_loaded_agents(OldList),
	% if Agt is already loaded
	member(Agt,OldList),
	% inform message
	write('Cannot load agent '+Agt+' : already loaded'),
        !,fail.

% predicate to remove an agent
remove_loaded_agent(Agt) :-
	list_loaded_agents(OldList),
	% if Agt is indeed loaded
	member(Agt,OldList),
	% remove it from the list
	findall(A,(member(A,OldList),A\=Agt),NewList),
	% retract old list
	retract(list_loaded_agents(OldList)),
	% assert new list instead
	assert(list_loaded_agents(NewList)),!.

% error message if trying to unload a not loaded agent
remove_loaded_agent(Agt) :- 
	list_loaded_agents(OldList),
	% if Agt is indeed loaded
	not(member(Agt,OldList)),
	% error message
	write('Cannot unload agent '+Agt+' : never loaded').
	
% remove all loaded agents
remove_all_loaded_agents :-
    list_loaded_agents(List),
    forall(member(Agt,List),remove_loaded_agent(Agt)),!.
    

%%%%%%%%%%%%%%%%%%%%%
%% erase from prolog all the content of an agent's KB
%% (to be used in GUI for reinitialise button
%%%%%%%%%%%%%%%%%%%%%
						
reinit_kb(A) :- 
	% remove from list of loaded agents
	remove_loaded_agent(A),
        nofail(retractallma(A)).
        
retractallma(A) :-
	% retract agent A's beliefs
	% commitments and ideals are special bels
	forall(bel(A,Prop,Deg,Time,Src),retract(bel(A,Prop,Deg,Time,Src))),
	% intentions
	forall(intend(A,Prop,Degprio,Time,Src),retract(intend(A,Prop,Degprio,Time,Src))),
	% desires
	forall(desire(A,Prop,Deg),retract(desire(A,Prop,Deg))),
	% undesires
	forall(undesire(A,Prop,Deg),retract(undesire(A,Prop,Deg))),
	% focus	
	forall(focus(A,Prop,Deg,Time),retract(focus(A,Prop,Deg,Time))),
	% emotions - FIXME: not asserted, are they?
	forall(emotion(A,Emo,Prop,Deg,Time),retract(emotion(A,Emo,Prop,Deg,Time))),
	% relationships (including trust?)
	forall(ami(A,B,D),retract(ami(A,B,D))),
	forall(ennemi(A,B,D),retract(ennemi(A,B,D))).

% TODO june2014 - use vecteur_bdi ou mental_attitude to find all MA to retract ?
%% add my_plans to the predicates that need to be retracted

%%%%%%%%%%%%%
%% LOAD KB %%
%%%%%%%%%%%%%

% warning: if a KB was reinitialised, all beliefs shared with all agents were also removed
% so need to reload all laws, action tendencies, commonsense beliefs, 
% as well as shared actions, and saturate KB
load_kb(A) :- 
	% add to list of loaded agents
	add_loaded_agent(A),!,
	% build name of KB file
	atom_concat('kbs/',A,X),
	atom_concat(X,'_bc',Y),
        % actually load the KB file
        [Y],
        % KB declares name of agent
        current_agent(Agt),
	[kbs/commonsense],
	[kbs/laws],
	[pleiad_tendencies],
	[pleiad_actions],
        debug_message(compil,['KB of agent ',A,' loaded successfully']),
        % saturate KB of this agent
        saturer_bc(Agt),!.

