 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% APRIL 2013                     %%
%% Carole Adam & Dominique Longin %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% todo: pouvoir interroger sur pride et shame sans specifier le groupe
% ie le predicat construit le groupe

%%%%%%%%%%%%%%%%%%%%%%%%%
%% DEFINITION OF SHAME %%
%%%%%%%%%%%%%%%%%%%%%%%%%

% shame = bel-i mutbel-group phi
%         and bel-i mutbel-group (phi -> not prop)
%         and bel-i mutbel-groupUi svalue-groupUi prop
%         and goal-i beLiked-i group

% WARNING: group MUST be ground or "out of local stack" error
shame(Agt,Group,Phi,Prop) :-
    instant(T),
    ground(Group),
    bel_mutbel(Agt,Group,Phi,T),
    bel_mutbel(Agt,Group,Phi -> not(Prop),T),
    % GroupU = union of Agt and Group, removing duplicates (if Agt was already in Group)
    list_to_set([Agt|Group],GroupU),
    bel_mutbel(Agt,GroupU,ideal(Agt,Prop,_DegIdeal),T),
    goal_beliked(Agt,Group).
 
% shame(Agt,Group,Phi,Prop) :-
%    % instant(T),
%    % FIXME TOMORROW : if Agt is not member of Group, needs a bel-i in front...
%    % so need bel-mutbel indeed...
%    mutualbel([Group],Phi),
%    mutualbel([Group],Phi -> not(Prop)),
%    mutualbel([Agt|Group]).
    
% should we have named groups with KBs of their own?
% that define their "official" group beliefs, values, etc?
% Group = [coco,kido,name=socio,mom],member(name=N,Group).


%%%%%%
%% SYMMETRIC DEF OF PRIDE
%%%%%%

% Phi implies Prop that is ideal for Agt wrt Group
pride(Agt,Group,Phi,Prop) :-
    instant(T),
    ground(Group),
    bel_mutbel(Agt,Group,Phi,T),
    bel_mutbel(Agt,Group,Phi -> Prop,T),
    % GroupU = union of Agt and Group, removing duplicates (if Agt was already in Group)
    list_to_set([Agt|Group],GroupU),
    bel_mutbel(Agt,GroupU,ideal(Agt,Prop,_DegIdeal),T),
    goal_beliked(Agt,Group).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% APPROX OF MUTUAL BELIEFS %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% mutualbel(+Group,+Prop,-List) returns in List the list of mental attitudes
% that must be true for the mutual belief to be true

%% approximation of mutual beliefs
% recursion on the group of agents
% base case: empty group -> true belief
mutualbel([],_Phi,[]).

% base case: single agent -> individual belief
mutualbel([Agt],Phi,[believe(Agt,Phi,_Deg,T)]) :- instant(T).

% recursive case using the auxbel auxiliary predicate
mutualbel(Group,Phi,List) :-
        mbaux(Group,[],Phi,List).
        
           
%    % for each agent1 in the group
%    forall(member(Agt1,Group),
%           % build the subgroup Group2 = Group - {Agt1}
%           % warning: only removes 1st occurrence, but Agt1 should only be once in Group anyway
%           (remove_agent_from_group(Agt1,Group,Group2),
%            % and check mutual beliefs of Agt1 with members Agt2 of this subgroup
%            auxbel2(Agt1,Group2,Phi,ListAgt1))).


%%%%%%%%%%%
%% AUX PRED FOR MUTBEL
%%%%%%%%%%%

% mbaux(Group,GroupDone,Phi,ListResult)
% - returns in result the list of mental attitudes of all agents
% about the beliefs of other agents in the group about Phi
% - always called on a non-empty group
% - GroupDone serves as the stop condition: when it is full with
% all agents from initial group, then it is finished

% base case : the group of treated agents is composed of all agents in the initial group
mbaux(Group1,Group2,_Phi,[]) :-
    % agents might not be in the same order so count them instead
    length(Group1,N),
    length(Group2,N),!.

% recursive case: treat next agent (Group is never empty)
mbaux([Agt|Group],Done,Phi,List) :-
    % ListA = beliefs of Agt wrt Group and Phi
    auxbel2(Agt,Group,Phi,ListA),
    % recursive call for beliefs of other agents
    % same group but treated agent goes to the end (must still be considered in beliefs of other agents about him)
    union(Group,[Agt],Group2),
    % but the treated agent is added to list of treated agents (to stop recursion when all are treated)
    mbaux(Group2,[Agt|Done],Phi,ListG),
    % final result is the union of beliefs of this agent and beliefs of the others in the group
    union(ListA,ListG,List).
    

%%%%%%%%%%
%% SUBGROUP
%%%%%%%%%%

% subgroup by removing Agt from Group
% warning: selectchk only removes first occurrence, so Agt should be only once in the group
% my predicate to remove all occurrences of agent in group
remove_agent_from_group(Agt,Group,Result) :-
    % remove first occurrence
    selectchk(Agt,Group,Interm),
    % then if Agt is still in group
    % member(Agt,Interm),
    % recursive call in case it is in the group more than once
    remove_agent_from_group(Agt,Interm,Result),!.

% group does not change if agt is not in it
remove_agent_from_group(Agt,Group,Group) :-
    not(member(Agt,Group)),!.
    
% my previous (and somewhat simpler) previous version of that
% remove_agent_from_group(Agt,Group,Result) :-
%    findall(Agt2,(member(Agt2,Group),Agt2\=Agt1),Group2).


%%%%%%%%%%
%% auxiliary predicate
%% beliefs of one member of the group
%% about Phi and about the beliefs of the other members
%%%%%%%%%%

% aux2 : bels of an agent wrt a group of agents
% vs an empty list: only Agt's own belief about Phi at current instant
auxbel2(Agt,[],Phi,[believe(Agt,Phi,_Deg1,T)]) :- instant(T),!.
        
auxbel2(Agt1,[Agt2|Group],Phi,
        % beliefs of Agt1 about the beliefs of Agt2
        [believe(Agt1,believe(Agt2,Phi,_Deg2,_T2),_Deg3,T),
        believe(Agt1,believe(Agt2,believe(Agt1,Phi,_Deg4,_T4),_Deg5,_T5),_Deg6,T),
        believe(Agt1,believe(Agt2,believe(Agt1,believe(Agt2,Phi,_DD1,_TT1),_DD2,_TT2),_DD3,_TT3),_DD4,_TT4)
        |List]) :-
        instant(T),
        % beliefs of Agt1 about the beliefs of the other agents in Group
        % -> recursive call
        auxbel2(Agt1,Group,Phi,List),!.

%%%%%%%%
%% bel_mutbel : belief of Agt about a mutual bel of Group about prop Phi
%% belief of an agent that there is a mutual belief in a group
%% cf in drafts/mutualbelief.pl
%%%%%%%%

% base case: this belief was asserted directly in the agent's KB
bel_mutbel(Agt,Group,Phi,T) :-
    believe(Agt,mutualbel(Group,Phi),_Deg,T).

% for the same group in a different order
% bel_mutbel(Agt,Group,Phi,T) :-
%    believe(Agt,mutualbel(Group2,Phi),_Deg,T),
%    % check it is in the same group only in different order of agents
%    subset(Group,Group2), subset(Group2,Group).
% --> subcase of next one

% or for any subgroup
bel_mutbel(Agt,Group,Phi,T) :-
    % to make it mutually exclusive
    not(believe(Agt,mutualbel(Group,Phi),_Deg1,T)),
    believe(Agt,mutualbel(Group2,Phi),_Deg2,T),
    % check if the mutual belief was asserted for a larger group containing the considered Group
    subset(Group,Group2).

% if agent is member of the group, he is aware of mutual belief
bel_mutbel(Agt,Group,Phi,_T) :-
    member(Agt,Group),!,
    % all the beliefs in the list are true
    forall((mutualbel(Group,Phi,List),member(Bel,List)),Bel).

% if agent is not member of the group
% it should have a belief that every belief composing the mutual belief of the group is true
bel_mutbel(Agt,Group,Phi,T) :-
    not(member(Agt,Group)),!,
    forall((mutualbel(Group,Phi,List),member(Bel,List)),
           believe(Agt,Bel,_Deg,T)).




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ABBREVIATIONS FOR SVALUE %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% group_svalue(Group,Prop) = MBel(Group,SValue(Group,Prop))
group_svalue(Group,Prop) :-
    % there is a mutual belief in the group that Prop is an ideal of each member of the group
    forall(member(Agt,Group),mutualbel(Group,ideal(Agt,Prop,_Deg))).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ABBREVIATION FOR BELIKED GOAL %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% in first approximation, we assume that agents have a goal to be liked by their friends
% being liked by a group = being liked by all members of this group
goal_beliked(Agt,Group) :-
    % if the group is instantiated
    ground(Group),
    % select subgroup without the agent itself
    remove_agent_from_group(Agt,Group,Subgroup),
    % just check that all members are friends
    forall(member(Agt2,Subgroup),ami(Agt,Agt2,_Deg)),!.

% RK: was using forall((member(Agt2,Group),Agt2\=Agt),ami(Agt,Agt2,_Deg))
% but problem when Agt is not instantiated, always true

% otherwise build the group by adding friends
% goal_beliked(_Agt,[]).
% goal_beliked(Agt,[Agt2|Group]) :-
%    ami(Agt,Agt2,_Deg),!,
%    %not(member(Agt2,Group)),
%    goal_beliked(Agt,Group).

% version that can construct a group: not working
% goal_beliked_c(Agt,GroupLiked) :-
%    findall(Ami,ami(Agt,Ami,_Deg),GroupAmi),
%    subset(GroupLiked,GroupAmi).







%%%%%%%%%%%%%%%%%%%%%%
%% SHAME DEFINITION %%
%%%%%%%%%%%%%%%%%%%%%%

% domi_shame(Agt,Group,Phi,Prop) :-
%    mutualbel()


