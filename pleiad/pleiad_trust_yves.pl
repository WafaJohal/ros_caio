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

% created July 9, 2013
% trust rules from pleiad_speechacts_benoit.pl

% in complement with Benoit's speech acts
% additional conditions and effects based on the trust between the agents

% rule(280)

:- kassert(trust(test,test,1)).

% declaration of rules for saturer_bc loop in pleiad_reasoning.pl
% trust rules in pleiad_trust_yves.pl rule(28)
rule(Index,T,Agt,Prem,Cci) :-
	trule(I,T,Agt,Prem,Cci),Index is 280+I.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% TRUST HANDLING RULES                       %%
%% rules to take trust and its 4 dimensions   %%
%% into account when deciding what to believe %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% cf Feltman - thin book of trust
% 4 dimensions of trust = competence, sincerity, reliability, care

%%%
% SINCERITY of agent A
%%%
% if it is grounded / it was expressed by agent A that it believes P
% then agents B who trust its sincerity will deduce that A really believes what it expressed

trule(1,T,B,
	% premises
	([
		% grounded for a group containing A and B that A believes P
		is_grounded([A,B],believe(A,P,DBel,TBel)),
		% B believes that A is sincere
		bel(B,sincere(A),DTrust,T,_Src)
	]),
	% conclusion
	% B deduces that A really believes P at the expressed degree
	% B's degree of belief depends on B's trust in A's sincerity
	bel(B,believe(A,P,DBel,TBel),DTrust,T,communication)
).

% so the agent can reason about the effect of the other's trust in him on the other's beliefs
trule(2,T,A,
	% premises
	([
		% A believes that B believes that he is sincere
                % ie A believes that B trusts his sincerity
		bel(A,believe(B,sincere(A),DTrust,T),DBel,T,_Src)
	]),
	% conclusion
        % A deduces that if it is grounded for a group containing A and B that A believes P
	%   then B will deduce (with degree = trust) that A really believes P at the expressed degree
	bel(A,grounded([A,B],believe(A,P,DBel,TBel))->believe(B,believe(A,P,DBel,TBel),DTrust,T),DBel,T,conviction)
).




%%%
% COMPETENCE
%%%

% 1) in beliefs
% what this agent believes is actually true

trule(3,T,B,
    % premises
    ([
        % B believes that A believes P
        believe(B,believe(A,P,DBel,_TBel),DBel2,T),
        % added July2013 to prevent B from adopting uninstantiated beliefs about values of props
        ground(P),
        % B believes that A is competent (about P?)
        bel(B,competent(A),DTrust,T,_Src),
        % merge all degrees
        DBel3 is DBel * DBel2 * DTrust
    ]),
    % conclusion
    bel(B,P,DBel3,T,communication)
).

% v2 implication

% trule(4,T,B,
%    % premises
%    ([
%        % B believes that A is competent (about P?)
%        bel(B,competent(A),DTrust,T,_Src) % ,
%        % merge all degrees - FIXME one degree not instantiated yet!
%        % DBelT is DBel * DTrust
%    ]),
%    % conclusion
%    % B believes that if A believes P, then P is true
%    bel(B,believe(A,P,_DBel,T)->P,DTrust,T,communication)
% ).
% commented 12July2013 because cannot force that P is grounded...
%                              and this creates problem with believe(value(prop,Val))


% 2) in intentions
% what this agent intends is feasible (A is capable of performing the actions promised)

trule(5,T,B,
    % premises
    ([
        % B believes that A believes P
        believe(B,believe(A,P,DBel,_TBel),DBel2,T),
        ground(P),
        % B believes that A is competent (about P?)
        bel(B,competent(A),DTrust,T,_Src),
        % merge all degrees
        DBel3 is DBel * DBel2 * DTrust
    ]),
    % conclusion
    bel(B,P,DBel3,T,communication)
).

%%%
% RELIABILITY
%%%
% when an intention of this agent is grounded, it is true
% ie when this agent expresses an intention, he actually has this intention

trule(6,T,B,
	% premises
	([
		% grounded for a group containing A and B that A intends P
		is_grounded([A,B],intend(A,P,DInt,TInt,Src)),
		% B believes that A is reliable
		bel(B,reliable(A),DTrust,T,_Src)
	]),
	% conclusion
	% B deduces that A really intends P at the expressed degree
	% B's degree of belief depends on B's trust in A's reliability
	bel(B,intend(A,P,DInt,TInt,Src),DTrust,T,communication)
).




%%%
% CARE
%%%
% this agent has the interests of others in mind when making decisions
% feels empathy?
% has a goal for the other's well being ? 
% hope that this agent will make choices that are advantageous for the others?

% if A cares for B, and A believes that B has a given intention, then if possible A will adopt B's intention
trule(7,T,A,
    % premises
    ([
        % A believes that B intends him to perform Action Alpha
        believe(A,intend(B,done(A,Alpha,_),DInt,_TInt,_Src),DBel,T),
        % A cares about B
        bel(A,care(B),DTrust,T,_Src),
        % merge all degrees
        % FIXME: need to do this extra check because for some reason there is an intention
        %        with uninstantiated degree that I cannot find where it comes from...
        ground(DInt),
        DAdopt is DBel * DInt * DTrust
    ]),
    % conclusion : A adopts B's intention
    intend(A,done(A,Alpha,_),DAdopt,T,trust)
).

% A can reason about this rule for another agent
% if A believes that B cares for A, that B believes that A has a given intention,
% then A deduces that if possible B will adopt A's intention
trule(8,T,A,
    % premises
    ([
        % A believes that B knows about A's intention that B performs Action Alpha
        believe(A,believe(B,intend(A,done(B,Alpha,_),DInt,_TInt,_Src),DBelB,T),DBelA,T),
        % A believes that B cares (about him)
        bel(A,believe(B,care(A),DTrust,T),DBelCare,T,_Src),
        % merge all degrees
        DAdInt is DBelB * DInt * DTrust,
        DAdBel is DBelA * DBelCare
    ]),
    % conclusion : A believes that B will adopt his intention
    bel(A,intend(B,done(B,Alpha,_),DAdInt,T,trust),DAdBel,T,communication)
).




%%%%%%%%%%%%%%%%%%
% FIXME MAY 2013
%%%%%%%%%%%%%%%%%%
% how to let the agent reason about these demo rules ?
% from trusting the other agent + intending to create a given belief
% he needs to infer intention to create the missing premise of the demo rule...
% moreover he must reason on the other agent's use of these rules
% ie A believes that B trusts him
% so A believes that if it is grounded for B that A believes Phi
% then B will believe that A believes Phi
% so A can deduce that by expressing his belief of Phi, he will create a belief in B that A believes Phi

