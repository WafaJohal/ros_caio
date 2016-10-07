
% intends to reach target
:- kassert(intend(robot,target,1,0)).

% transporting box will get it to the target location
:- kassert(bel(robot,after(robot,transport,target),1,0,conviction)).

% intention to transport will be deduced via planning
%:- kassert(intend(robot,done(robot, transport,_),1,0)).

% if robot has alternative action available (with lower probability so selected next)
%:- kassert(bel(robot,after(robot,send,target),0.7,0,conviction)).
% it can cope by resigning its first intention (transport) and change to this new one (sending)

% but transport empties the battery
:- kassert(bel(robot,after(robot, transport, emptybatt),0.7,0,conviction)).

% staying in place does not
%:- kassert(bel(robot,after(robot, transport, not(emptybatt)),0.7,0,conviction)).
% but does not reach intention either


% undesirable to empty battery
:- kassert(undesire(robot, emptybatt,1)).