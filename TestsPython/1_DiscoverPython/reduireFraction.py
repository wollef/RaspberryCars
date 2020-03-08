
def calculePGCD(na,nb):
   ia = na
   ib = nb
   while ib !=0 :
       itemp = ia
       ia = ib
       ib = itemp % ib
   return ia


#lit les nombres.
inputNumerateur = input ("Entre le numérateur:")
numerateur = int(inputNumerateur)
inputDenominateur = input ("Entre le dénominateur:")
denominateur = int(inputDenominateur)
print ("la saisie est :",numerateur,"/",denominateur)

# calcul du pgcd en utilisant le programme d'exemple
pgcd = calculePGCD (numerateur , denominateur)

#calcul du nouveu numérateur et du nouveau dénominateur
numerateur = (int) (numerateur / pgcd)
denominateur = (int) (denominateur / pgcd)

# affiche le resultat
print ("le resultat est :",numerateur,"/",denominateur)

