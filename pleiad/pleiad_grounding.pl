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

%% Created July 9, 2013
%% anything about grounding
%% from pleiad_speechacts_benoit.pl

% predicate that should be asserted: grounded
% predicate to be interrogated: is_grounded
%          --> checks both direct grounding (grounded predicate asserted)
%                          and indirect grounding (same group in different order)

:- dynamic(grounded/2).
:- dynamic(assert_grounding/2).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ASSERTION OF GROUNDING %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

assert_grounding(Group,Prop) :-
    if_debug_tag(ground,writeln('assert new grounding, group='+Group+', prop='+Prop)),
    assert(grounded(Group,Prop)).
    % idea : also assert the same for each permutation group?



%%%%%%%%%%%%%%%%%%%%%%%
%% LOGIC OF GROUNDED %%
%%%%%%%%%%%%%%%%%%%%%%%

% the order of agents in the group does not matter
% this call must be able to GENERATE a group of agents in which Phi is grounded
% (so that demo rules with a grounded in their premises can work)
% grounded(K,Phi) :-
%    ground(K), % if commented : out of local stack
%                % if not commented : does not find other orders of agents in the group
%    % ground(Phi),
%    grounded(K2,Phi),
%    subset(K,K2),subset(K2,K).


% for generating permutations
indirect_grounded(K,Phi) :-
    %ground(Phi),
    % find all groups GK
    findall(GK,
            % that are a permutation of a group GK1 in which Phi is grounded
            (direct_grounded(GK1,Phi),permutation(GK1,GK),GK1\=GK),LK),
    % remove doubles from the list of permutations
    list_to_set(LK,SK),
    % check that the group requested is a member of this set of permutations
    member(K,SK).

% direct grounded = the fact is asserted in KB
% this prevents infinite loop of indirect_grounded calling indirect_grounded
direct_grounded(K,Phi) :- grounded(K,Phi).

% grounded predicate to be used to query the KB, different from the predicate that is asserted
is_grounded(K,Phi) :- direct_grounded(K,Phi).
is_grounded(K,Phi) :- indirect_grounded(K,Phi).

%    %ground(K),
%    findall(GK1,(ground(GK1),grounded(GK1,Phi),permutation(GK1,GK)),LK),
%    member(K,LK).
%    %permutation(K,KP),K\=KP,
%    %grounded(KP,Phi).

% grounded-k phi -> grounded dans tout sous groupe de k que grounded-k phi
% subset(Sub,Set), returns only []  but check membership if subset and set are ground


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% SPECIAL CASE - PERFORMANCE OF ACTIONS             %%
%%                                                   %%
%% HYP : agent have perfect and complete perception  %%
%% ie grounded(done) -> bel(done)                    %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

rule(31,T,Agt,
     % premise
     [
        % performance of some action by some agent at some time
        % is gorunded in some group G
        is_grounded(G,done(Someone,Someaction,Sometime)),
        % and Agt is a member of that group
        member(Agt,G)
    ],
    % conclusion : this agent believes that the action was performed
    bel(Agt,done(Someone,Someaction,Sometime),1,T,observation)
).

