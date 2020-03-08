
def calculePGCD(na,nb):
   ia = na
   ib = nb
   while ib !=0 :
       itemp = ia
       ia = ib
       ib = itemp % ib
   return ia

a = 55
b = 33
resultat =calculePGCD(a,b)
print("le pgcd de",a,"et",b,"est", resultat)
