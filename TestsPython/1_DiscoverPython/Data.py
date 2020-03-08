class Personne:
    nom=""
    prenom=""
    annee=0
    
    def __init__(self, n,p,a ):
        self.nom = n;
        self.prenom = p;
        self.annee =a;
        
    def age(self) :
        return 2019-self.annee;
    
    def affiche(self):
        print("NOM:",self.nom);
        print("PRENOM:",self.prenom);
        print("ANNEE:",self.annee);
        print("AGE:",self.age());
        print();
    
personnes = [];    
personnes.append(Personne("leblond","yves",1972));
personnes.append(Personne("leblond","thibault",2004));
personnes.append(Personne("leblond","amaury",2006));
personnes.append(Personne("leblond","maxime",2007));

for p in personnes :
    p.affiche();
