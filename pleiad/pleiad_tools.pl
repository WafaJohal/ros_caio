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

%%%
% Ideas TODO October 2012
%  - tool to modify degrees (focus or other, is there any other?) cf time.pl
%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% tools  %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% conditions and no-fails %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% conditional instruction
%
ifthenelse(A,B,C) :- (A,B);C. 
ifthen(A,B) :- ifthenelse(A,B,true). 


%% non-failing instruction sequence 
%% do A if possible, but then do B anyway (even if A failed)
%
ifpossible(A,B) :- ifthenelse(A,B,B). %

%% no-fail instruction
%% do A then continue even if A failed
%
nofail(A) :- A;true.




%%%%%%%%%%%%%%%%%%%%%%
%% agents vs humans %%
%%%%%%%%%%%%%%%%%%%%%%

artificial_agent(Agt) :-
    ground(Agt),
    atom_concat('kbs/',Agt,X),
    atom_concat(X,'_bc.pl',Y),     
    exists_file(Y).
    
human_agent(Agt) :-
    ground(Agt),
    atom_concat('kbs/',Agt,X),
    atom_concat(X,'_bc.pl',Y),     
    not(exists_file(Y)).

%%%%%%%%%%%%%%%%%%%%%%%%%%
%% asserts and retracts %%
%%%%%%%%%%%%%%%%%%%%%%%%%%

%% assertion after retracting previous similar assertions
%
kassert(P) :- (retract(P),assert(P)),!.
kassert(P) :- assert(P). 


%% retract a bel and recover the previous degree
%% without failing if this bel() did not exist yet (in that case return degbel=0)
%

% normal case: retract previous bel and get its degree
kretract(bel(Agt,Prop,Degbel,Time,Type),Degbel) :- 
			retract(bel(Agt,Prop,Degbel,Time,Type)),!.

% exception case: nothing to retract, previous degree returned is 0
kretract(bel(_Agt,_Prop,_Degbel,_Time,_Type),0) :- !.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DEFINITION FILE OF A PREDICATE %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% for ex, Pred = rule(_,_,_,_,_)
def_file(Pred,F) :- predicate_property(Pred,file(F)).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% BELIEFS SPLIT FUNCTION %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Moved to pleiad_beliefs.pl October 2012




%%%%%%%%%%%%%%%%%%%%%%%
%% sorting functions %%
%%%%%%%%%%%%%%%%%%%%%%%

%% IDEA TODO Octoer 2012
% just define an ordering function to compare two elements (eg 2 bels, 2 emotions, etc)
% and use it in one generic sorying function that compares the two head elements to interleave the 2 lists
%



%%%%%%%%%%%%%%
% generic sorting function
% using specific comparing functions
%%%%%%%%%%%%%%

% EN COURS

% insert non-sorted list L1 into sorted list L2 to form sorted list Rez
insert_list([],L2,L2). 
insert_list([E1|L1],L2,Rez) :- insert_one(E1,L2,Aux),insert_list(L1,Aux,Rez).

% insert one element E in its place in a sorted list L to form result list
insert_one(E1,[],[E1]).
insert_one(E1,[E2|L2],[E1|[E2|L2]]) :- compare_sup(E1,E2),!. 
insert_one(E1,[E2|L2],[E2|Aux]) :-
			%D1=<D2,
			insert_one(E1,L2,Aux).

% sorting a vector of emotions = insert it in order in an empty list
sort_vector(Vect,Vsort) :- insert_list(Vect,[],Vsort).

% abbreviations for retro-compatibility
sort_emo(Vemo,Vsort) :- sort_vector(Vemo,Vsort),!.
sort_bel(Vbel,Vsort) :- sort_vector(Vbel,Vsort),!.
sort_des(Vdes,Vsort) :- sort_vector(Vdes,Vsort),!.
sort_intend(Vintend,Vsort) :- sort_vector(Vintend,Vsort),!. 
sort_goal(V,Vsort) :- sort_vector(V,Vsort),!.

%%%%%%%%%%%%%%
% generic compare function
% base case : comparing uncomparable elements fails
%%%%%%%%%%%%%%

% EN COURS

% 2 emotions by intensity degree
compare_sup(emotion(_A1,_E1,_P1,D1,_T1),emotion(_A2,_E2,_P2,D2,_T2)) :- D1>D2,!.

% 2 beliefs by belief degree
compare_sup(bel(_A1,_P1,D1,_T1,_S1),bel(_A2,_P2,D2,_T2,_S2)) :- D1>D2,!.

% desires and undesires interleaved in a single vector by intensity degree
compare_sup(desire(_A1,_P1,D1),desire(_A2,_P2,D2)) :- D1>D2,!.
compare_sup(desire(_A1,_P1,D1),undesire(_A2,_P2,D2)) :- D1>D2,!.
compare_sup(undesire(_A1,_P1,D1),undesire(_A2,_P2,D2)) :- D1>D2,!.
compare_sup(undesire(_A1,_P1,D1),desire(_A2,_P2,D2)) :- D1>D2,!.

% intention by priority
compare_sup(intend(_A1,_P1,D1,_T1,_S1),intend(_A2,_P2,D2,_T2,_S2)) :- D1>D2,!.

% goals by degree
compare_sup(goal(_A1,_P1,D1,_T1,_S1),goal(_A2,_P2,D2,_T2,_S2)) :- D1>D2,!.

% all other comparisons fail (including comparisons between uncomparable elements)



%%%%%%%%%%%%%%
%%% COPING %%%
%%%%%%%%%%%%%%

% TODO October 2012 : re-program it using the generic sorting function?

% sort parameters of a coping strategy by degree of a belief linked to this param
% param 1: vector of pairs (Param,Deg)
% return = vector of pairs (Param,Deg) ranked by Deg

tripardeg_croissant(VPaires,VSorted) :- insertlistepaire_croissant(VPaires,[],VSorted).

insertlistepaire_croissant([],L,L).
insertlistepaire_croissant([Paire|ListePaires],L,LP) :- insertpaire_croissant(Paire,L,Laux),
											 insertlistepaire_croissant(ListePaires,Laux,LP).

insertpaire_croissant(A,[],[A]).
insertpaire_croissant((Param1,Deg1),[(Param2,Deg2)|LP],[(Param1,Deg1),(Param2,Deg2)|LP]) :-
					Deg1<Deg2.
insertpaire_croissant((Param1,Deg1),[(Param2,Deg2)|LP],[(Param2,Deg2)|LP2]) :-
					Deg1>=Deg2,insertpaire_croissant((Param1,Deg1),LP,LP2).
					
tripardeg_decroissant(VPaires,VSorted) :- insertlistepaire_decroissant(VPaires,[],VSorted).

insertlistepaire_decroissant([],L,L).
insertlistepaire_decroissant([Paire|ListePaires],L,LP) :- insertpaire_decroissant(Paire,L,Laux),
											   insertlistepaire_decroissant(ListePaires,Laux,LP).
											 
insertpaire_decroissant(A,[],[A]).
insertpaire_decroissant((Param1,Deg1),[(Param2,Deg2)|LP],[(Param1,Deg1),(Param2,Deg2)|LP]) :-
					Deg1>Deg2.
insertpaire_decroissant((Param1,Deg1),[(Param2,Deg2)|LP],[(Param2,Deg2)|LP2]) :-
					Deg1=<Deg2,insertpaire_decroissant((Param1,Deg1),LP,LP2).


%%%%%%%%%%%%%%%%%%%%%%%%
%% LISTS MANIPULATION %%
%%%%%%%%%%%%%%%%%%%%%%%%

% delete element in a list
effacer(_,[],[]). 
effacer(X,[X|Y],Z) :- effacer(X,Y,Z),!.
effacer(X,[Y|Z],[Y|T]) :- effacer(X,Z,T).

% delete list of elements in another list
deliste([],L,L) :- !. 
deliste([X|Y],Z,T) :- effacer(X,Z,U),deliste(Y,U,T).

% inversion of plan
inverse_plan([],[]).
inverse_plan([A|Plan],Plan2) :- 
		inverse_plan(Plan,Plan1),
		append(Plan1,[A],Plan2).
                		
% not used anymore - probably still useful in the future!
append_no_repeat([],L2,L2).
append_no_repeat([E1|L1],L2,L) :-  member(E1,L2),append_no_repeat(L1,L2,L),!.
append_no_repeat([E1|L1],L2,[E1|L]) :- not(member(E1,L2)), append_no_repeat(L1,L2,L).


%% NEW MAY 2013

%% generate permutations of lists

list_permutations([],[[]]).
list_permutations([A],[[A]]).
list_permutations(L,LL) :- findall(LP,permutation(L,LP),LL).

%% insert elements in lists

insert_first_position(Elt,List,NewList) :-
    nth0(0,NewList,Elt,List),!.
    
insert_last_position(Elt,List,NewList) :-
    length(List,N),
    nth0(N,NewList,Elt,List),!.


%%%%%%%%%%%%%%%%%%%%%%
%% clauses to list  %%
%%%%%%%%%%%%%%%%%%%%%%

clause_list(Predicate,List) :-
        % get body of Predicate
        clause(Predicate,Clauses),
        clauses_to_list(Clauses,List).
        

% did not find a simpler way than recursive until find the correct number of clauses in body...
clauses_to_list((C1,C2,C3,C4,C5,C6,C7,C8,C9,C10),[C1,C2,C3,C4,C5,C6,C7,C8,C9,C10]) :- !.
clauses_to_list((C1,C2,C3,C4,C5,C6,C7,C8,C9),[C1,C2,C3,C4,C5,C6,C7,C8,C9]) :- !.
clauses_to_list((C1,C2,C3,C4,C5,C6,C7,C8),[C1,C2,C3,C4,C5,C6,C7,C8]) :- !.
clauses_to_list((C1,C2,C3,C4,C5,C6,C7),[C1,C2,C3,C4,C5,C6,C7]) :- !.
clauses_to_list((C1,C2,C3,C4,C5,C6),[C1,C2,C3,C4,C5,C6]) :- !.
clauses_to_list((C1,C2,C3,C4,C5),[C1,C2,C3,C4,C5]) :- !.
clauses_to_list((C1,C2,C3,C4),[C1,C2,C3,C4]) :- !.
clauses_to_list((C1,C2,C3),[C1,C2,C3]) :- !.
clauses_to_list((C1,C2),[C1,C2]) :- !.
clauses_to_list((C1),[C1]) :- !.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% product of degrees in a list of degrees %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

product_list_degrees([Deg],Deg) :- !.
product_list_degrees([Deg|List],Deg2) :- product_list_degrees(List,Deg1),Deg2 is Deg * Deg1.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DISPLAY MESSAGES ------- %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% display (print) a text message (list of strings)

display_message([]) :- print('\n\n'),!.
display_message([A]) :- atom(A),print(A),print('\n\n'),!.
display_message([L]) :- is_list(L),display_list(L), !.
display_message([A|L]) :- print(A), display_message(L).

display_list([A]) :- print(A).
display_list([A|L]) :- print(A),print(' and '),display_list(L).
