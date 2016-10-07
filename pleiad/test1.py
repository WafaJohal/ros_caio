from pyswip.prolog import Prolog
from pyswip.easy import getList, registerForeign
from pyswip import *


prolog = Prolog()
p=prolog.consult("test.pl")
assertz = Functor("asserta", 1)
perception = Functor("perception",2)

X = Variable()
q = Query(perception("nao", X))
while q.nextSolution():
    print X.value
q.closeQuery()

call(assertz(perception("nao","wafa")))
#prolog.assertz("perception(nao,wafa)")
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
