from pyswip.prolog import Prolog
from pyswip.easy import getList, registerForeign
from pyswip import *

prolog = Prolog()
p=prolog.consult("pleiad.pl")
assertz = Functor("assertz", 1)
position = Functor("position",2)
save_time = Functor("save_time",1)
#nao=Functor("nao")
#b=call(nao)
#print b
prolog.query("nao.")
#call("nao")

X = Variable()
q = Query(position("naomjk", X))
while q.nextSolution():
    print X.value
q.closeQuery()


prolog.assertz("position(naomjk,mjk)")


X = Variable()
q = Query(position("naomjk", X))
while q.nextSolution():
    print X.value
q.closeQuery()


#proactive=Functor("agent_proactive_loop",2)

call(save_time("naomjk"))

#X = Variable()
#q = Query(proactive("naomjk",X))
#while q.nextSolution():
#    print X.value
#q.closeQuery()
#call(proactive("naomjk",Action))

