%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                                                       %%
%% PLEIAD : ProLog Emotional Intelligent Agents Designer %%
%%                                                       %%
%% Author : Carole Adam                                  %%
%% Version : v7 - June 2014                              %%
%%                                                       %%
%%    ------- DO NOT DISTRIBUTE -------                  %%
%%                                                       %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% moved out of pleiad_tools because needs kassert predicate

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TRACE / UNTRACE             %%
% AND OTHER DEBUG FACILITIES  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% from pleiad.pl

%%%%
% shortcuts
%%%%

% see help(trace) for more
untrace(Pred) :- trace(Pred,-all).
traceall(Pred) :- trace(Pred,+all).
tracefail(Pred) :- trace(Pred,+fail).
stop_trace :- notrace.
stop_debug :- nodebug.
info_debug :- debugging.


%%%%
% old version of debug: single value
%%%%

% toggle true/false to activate debug
% debug mode prints out all demo rules applied when saturating KBs
% :- assert(kdebug(false)).

% switchdebug(on) :- retract(kdebug(_)),assert(kdebug(true)),!.
%%% switchdebug(on) :- print('debug already on').

% switchdebug(off) :- retract(kdebug(_)),assert(kdebug(false)),!.
%%% switchdebug(off) :- print('debug already off').

%%% personalised value to switch debug for some files only
%setdebug(Value) :- retract(kdebug(_)),assert(kdebug(Value)),!.

%%% ifdebug
%%% debug predicate toggled in pleiad.pl
% ifdebug(A) :- kdebug(true),A.
% ifdebug(_A) :- kdebug(Other), Other \= true.
% ifdebugval(Val,A) :- kdebug(Val),A.
% ifdebugval(Val,_A) :- kdebug(Other),Other \= Val.


%%%%
%% new version : list of debug tags
%%%%


% add a debug tag to the list
add_debug_tag(Tag) :-
    % get the list
    nofail(retract(kdebugtags(List))),
    % add the tag and remove doubles
    list_to_set([Tag|List],Set),
    assert(kdebugtags(Set)),!.
    
add_debug_tags([]) :- !.
add_debug_tags([A|B]) :- add_debug_tag(A),add_debug_tags(B),!.
    
% remove a debug tag from the list (if present, do not fail if absent)
remove_debug_tag(Tag) :-
    % get the list
    retract(kdebugtags(List)),
    % delete element in the list
    delete(List,Tag,New),
    assert(kdebugtags(New)).
    
% check if a debug tag is in the list
if_debug_tag(Tag,Todo) :-
    kdebugtags(List),
    member(Tag,List),Todo.
% do not fail if not in debug mode!
if_debug_tag(Tag,_Todo) :-
    kdebugtags(List),
    not(member(Tag,List)).
    % do nothing
    
% debug message
debug_message(Tag,Msg) :-
    append(Msg,['\n\n'],NMsg),
    if_debug_tag(Tag,display_message(NMsg)).
