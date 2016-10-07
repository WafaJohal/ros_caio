from pyswip.prolog import Prolog
from pyswip.easy import getList, registerForeign
from pyswip import *

prolog = Prolog()
p=prolog.consult("test.pl")
prolog.consult("pleiad.pl")
prolog.query("nao").
assertz = Functor("assertz", 1)
perception = Functor("perception",2)
bel = Functor("believe",2)
writebel = Functor("write_beliefs",3)
goal = Functor("goal",5)
save_time = Functor("save_time",2)

X = Variable()
q = Query(perception("nao", X))
while q.nextSolution():
    print X.value
q.closeQuery()

call(assertz(perception("nao","john")))
#prolog.assertz("perception(nao,john)")
call(writebel("nao",0,"test.pl"))

prolog.consult("test.pl")

variable = raw_input('input something!: ')

X = Variable()
q = Query(perception("nao", X))
while q.nextSolution():
    print X.value
q.closeQuery()


variable = raw_input('input something!: ')


X = Variable()
q = Query(perception("nao", X))
while q.nextSolution():
    print X.value
q.closeQuery()
