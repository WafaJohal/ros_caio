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

% tag for debug : focus

debug_focus :- trace(set_focus),add_debug_tag(focus).

%% Rk October 2012 : add functions to decay focus automatically over time
% and to manage ontological relations between concepts in prop content
%% --> these functions exist in pleiad_time.pl

% new July 2013: transmission of focus over network of consequences

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% focus definition %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% moved here from pleiad_time.pl (October 2012)
% focus is used in emotions triggering and coping strategies

% focus is the predicate that is asserted, infocus is the shortcut to interrogate it
% with specific cases added for negations, compositions, etc

% something is in focus if its degree of focus is ?
% old version: at least 0.5 - but danger of a too high threshold (eg 0.5) : nothing is triggered
% new version: existant
%
infocus(Agt,Prop,D,T) :- focus(Agt,Prop,D,T). %,D>0.5.
infocus(Agt,not(Prop),D,T) :- focus(Agt,Prop,D,T).

% new July 2013 - only problem : it generates infocus for props that are not supposed to have values...
% arguments not sufficiently instantiated FIXME 5JULY2013
% degree was not instantiated
infocus(Agt,value(Prop,_Val),D,T) :- instant(T),ground(Agt),ground(Prop),focus(Agt,Prop,D,T).
infocus(Agt,value(_Prop,Val),D,T) :- instant(T),ground(Agt),ground(Val),focus(Agt,Val,D,T).





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%   focus change   %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%
% voluntary modification of focus
% that will be used in various coping strategies
% and also when adding a mental attitude in KB (which activates its propositional content)
%%%

% case of negative proposition: set focus on the atom
set_focus(Agt,not(Prop),Deg,T) :- set_focus(Agt,Prop,Deg,T). 

% case of done(action) : focus on the action
set_focus(Agt,done(_Someone,Action,_Sometime),Deg,Time) :-
        set_focus(Agt,Action,Deg,Time).

% set focus on a list (cprop of expressive speech act = [emotion, object, intensity] )
set_focus(Agt,[Emo,EmoObject,_EmoDeg],Deg,Time) :-
        % check it is the cprop of an expressive
        emotiontype(Emo),
        % if it is: set focus on the object of the emotion (not on the emotion itself)
        set_focus(Agt,EmoObject,Deg,Time).

% set focus on a list (object of an emotion, can contain one or 2 propositions / actions / events )
set_focus(_Agt,[],_Deg,_Time) :- !.
set_focus(Agt,[Prop|List],Deg,Time) :-
        % set focus on first element of list
        set_focus(Agt,Prop,Deg,Time),
        % and recursively on all elements of list
        set_focus(Agt,List,Deg,Time).

% set focus on the value of a prop : set focus on the prop and its value? FIXME

% set focus on a future prop = set focus on that prop? FIXME


% set focus on a mental attitude = set focus on its object / cprop
% pb: how to get it?


% basic case : retract previous focus on same prop before asserting new focus
%              and do not assert focus on a mental attitude
set_focus(Agt,Prop,Deg,T) :-
		% only set focus on atomic propositions
		% atom(Prop),
                % APRIL 2013 - allow non atomic propositions
                % on condition that they are not mental attitudes of any agent (cf pleiad_kbmgr.pl)
                not(mental_attitude(_AnyAgt,Prop)),
		% try to retract previous value of focus
		ifpossible(retract(focus(Agt,Prop,_,_)),
		% before asserting new value
		assert(focus(Agt,Prop,Deg,T))),
                if_debug_tag(focus,writeln('set focus of agent '+Agt+' on proposition '+Prop+' to degree '+Deg+' at time '+T)),
                % recursively set focus on consequences as well
                set_focus_rec(Agt,Prop,Deg,T),
                !.

% set focus must not fail
set_focus(Agt,Prop,_,_) :- if_debug_tag(focus,writeln('set focus default: do nothing, agent='+Agt+' prop='+Prop)),!.


% recursively set focus on the consequences of propositions
% (prob is an abbrev of bel, but could also be asserted directly)
% set_focus_rec(Agt,Prop,Deg,Time) :-
%    findall(Csq1,bel(Agt,Prop->Csq1,DBel,TBel,SBel),ListBel),
%    findall(Csq2,prob(Agt,Prop->Csq2,DProb,TProb),ListProb),
%    merge(ListBel,ListProb,ListCsq),
%    set_focus_rec()

set_focus_rec(Agt,Prop,Deg,Time) :-
    forall((bel(Agt,Prop->Csq1,DBel,_TBel,_SBel),NewDeg is Deg*DBel*0.8),set_focus(Agt,Csq1,NewDeg,Time)).



%%%%%%%%%%%%%%%%%%
%%%%% TOOLS  %%%%%
%%%%%%%%%%%%%%%%%%

%%%%%%%%
% power : recursive def
% actually use ** operator...
%%%%%%%%

%power(_,0,1) :- !.
%power(N,1,N) :- !.
%power(N1,N2,N) :- NN is N2 -1, power(N1,NN,N3), N is N1*N3,!.
 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% tools to increase focus ------ %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% pb: stuck in 0
increase_focus_coeff(0,0.01,_Coeff) :- !.
increase_focus_coeff(OldFocus,NewFocus,Coeff) :- 
	C2 is 1/Coeff, NewFocus is OldFocus**C2.

augmenter_degre_coeff(Deg,Newdeg,Coeff) :- Newdeg is (Deg+Coeff)/(Coeff+1). 
augmenter_degre_immense(Deg,Newdeg) :- augmenter_degre_coeff(Deg,Newdeg,4).
augmenter_degre_beaucoup(Deg,Newdeg) :- augmenter_degre_coeff(Deg,Newdeg,3).
augmenter_degre_moyen(Deg,Newdeg) :- augmenter_degre_coeff(Deg,Newdeg,2).
augmenter_degre_unpeu(Deg,Newdeg) :- augmenter_degre_coeff(Deg,Newdeg,1).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% tools to decrease focus ----- %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% pb: stuck in 1
decrease_focus_coeff(1,0.99,_Coeff) :- !.
decrease_focus_coeff(OldFocus,NewFocus,Coeff) :-
	NewFocus is OldFocus**Coeff.

% warning: the lower the degree the bigger the decrease!
diminuer_degre_coeff(Deg,Newdeg,Coeff) :- Newdeg is (Deg*Coeff)/(Coeff+1). 
diminuer_degre_unpeu(Deg,Newdeg) :- diminuer_degre_coeff(Deg,Newdeg,4).
diminuer_degre_moyen(Deg,Newdeg) :- diminuer_degre_coeff(Deg,Newdeg,3).
diminuer_degre_beaucoup(Deg,Newdeg) :- diminuer_degre_coeff(Deg,Newdeg,2).
diminuer_degre_immense(Deg,Newdeg) :- diminuer_degre_coeff(Deg,Newdeg,1).
