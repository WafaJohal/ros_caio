:- kassert(desire(tom,happy,1)).

% necessary to define predicate abandoned (todo: place it in pleiad)
:- kassert(abandoned(tom,nothing,0)).


%% mutual beliefs
:- kassert(bel(tom,not(rangee),1,0,observation)).
%:- kassert(bel(tom,believe(mamtom,not(rangee),1,0),1,0,conviction)).
%:- kassert(bel(tom,believe(mamtom, believe(tom,not(rangee),1,0),1,0),1,0,conviction)).


%% mutual beliefs with his mother: direct assertion
:- kassert(bel(tom,mutualbel([tom,mamtom],not(rangee)),1,0,observation)).
:- kassert(bel(tom,mutualbel([tom,mamtom],not(rangee)->not(autonome)),1,0,conviction)).
:- kassert(bel(tom,mutualbel([tom,mamtom],ideal(tom,autonome,1)),1,0,conviction)).

% FIXME: ideal used on actions and props... inconsistent

%% mutual beliefs with his friends
:- kassert(bel(tom,mutualbel([tom,kenzo,maxim,lila],not(rangee)),1,0,observation)).
:- kassert(bel(tom,mutualbel([tom,kenzo,maxim,lila],not(rangee)->cool),1,0,conviction)).
:- kassert(bel(tom,mutualbel([tom,kenzo,maxim,lila],ideal(tom,cool,1)),1,0,conviction)).

%% TODO next: event to a group of observer should assert mutual belief in this group
%% that the consequence of the event is true


%% amis
:- kassert(ami(tom,mamtom,1)).
:- kassert(ami(tom,kenzo,1)).
:- kassert(ami(tom,maxim,1)).
:- kassert(ami(tom,lila,1)).



%%%%%%%%%%%%%%
%% SCENARIO %%
%%%%%%%%%%%%%%

%1% stimulus([tom,maxim,lila,kenzo,mamtom],event(annonce,not(rangee))).
%2% ?- pride(tom,[lila,kenzo,maxim],not(rangee),Prop).
%%  Prop = cool.
%3% ?- shame(tom,[mamtom],not(rangee),Prop).
%% Prop = autonome.





