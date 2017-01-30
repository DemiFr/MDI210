package descente.modele;

import java.util.ArrayList;
import java.util.Observable;

import probleme.Pb;

/**
 * Classe principale qui modelise la methode de descente
 */
public class Descente extends Observable implements Runnable  {

	private double seuil = 0.00001; // si le gradient a une longueur inferieure a seuil, la methode de descente s'arrete
	private Pb pb;  // le probleme traite
	private Couple P = null;    // doit contenir le point courant du plan dans la methode de descente
	private Domaine domaine;    // le domaine dans lequel on cherche le minimum de la fonction
	private int nbPas = 0;      // sert a  compter le nombre de pas de la methode de descente
	private Couple direction;   // pour la direction que s'apprete a  suivre la methode de descente
	private boolean suspendre;  // sert a suspendre la descente
	private boolean finie;       // vaut faux pendant que la descente s'effectue
	private boolean stoppee;	// passe a  vrai si la methode de descente est interrompue par l'utilisateur
	private boolean atteintMinimum = true;  // passe a faux si la methode de descente montre que le probleme n'est pas borne
	private int delai = 2000;    /* sert a ralentir la descente pour l'affichage graphique ; 
								    delai en millisecondes entre deux etapes de la descente */

	public static double epsilon = 1E-12;

	/**
	 * Un double est considere comme valant 0 si sa valeur absolue est inferieure a epsilon
	 * @param v le double considere
	 * @return true si le nombre est considere comme nul, false dans le cas contraire
	 */
	public static boolean estNul(double v) {
		return v < epsilon && v > -epsilon;
	}

	/**
	 * @param pb le probleme considere ; le probleme consiste toujours a  chercher le minimum
	 *  d'une fonction convexe de deux variables sur un domaine du plan limite par des demi-droites.
	 */
	public Descente(Pb pb)  {
		this.pb = pb;
		this.domaine = pb.getDomaine();
	}

	/**
	 * Fait trois actions : 
	 * <br>- notifie les observateurs, ici l'interface graphique, afin que celle-ci actualise l'affichage 
	 * <br>- suspend le thread en cours d'execution pendant un nombre minimum de millisecondes egal a la valeur de delai
	 * <br>- si la methode est suspendue, attend jusqu'a  recevoir une notification alors que la methode n'est plus 
	 * suspendue.  
	 */
	public synchronized void prevenirEtAttendre() {
		this.setChanged();
		this.notifyObservers();
		try {
			this.wait(delai);
			while (suspendre) wait();
		}
		catch (Exception exc) {
			exc.printStackTrace();
		}	
	}

	/**
	 * La methode de descente est lancee dans un thread faisant tourner cette methode run.
	 */
	public void run() {
		if (P != null) effectuer();
	}

	/**
	 * Effectue la descente a  partir du point P.
	 */
	public void effectuer() {
		// un passage dans la boucle pour chaque etape de la methode de descente
		do {
			this.prochaineDirection();
			this.prevenirEtAttendre(); 
			if (!this.finie)	 {
				this.P = this.pointSuivantDescente();
				if (this.P == null) { // la fonction f n'atteint pas de minimum sur le domaine
					this.atteintMinimum = false;	
					this.finie = true;
				}
			}
			nbPas++;
		} while (!this.finie && !this.stoppee);
		this.prevenirEtAttendre(); 
	}

	/*/**<font color="red">
	 * METHODE A IMPLEMENTER Recherche la prochaine direction a suivre lorsque le point courant P
	 * est dans l'interieur strict du domaine.
	 * Si la descente n'est pas finie, met la prochaine direction dans l'attribut this.direction  
	 * (c'est obligatoire d'utiliser cet attribut).
	 *<br> ATTENTION : si la methode de descente est finie, la methode doit passer this.finie a true.
	 * @see Couple#norme()
	 * @see Couple#mult(double)
	 * @see Pb#gradientf(Couple)
	 * @see #seuil
	 </font>*/
	private void directionSiInterieur() {		
		Couple dir = pb.gradientf(P);
		if(dir.norme()> seuil)
			this.direction = dir.mult(-1);
		else// norme trop faible, terminer le program
		{
			this.direction =new  Couple(0,0);
			this.finie = true;
		}
	}

	/**<font color="red">
	 * METHODE A IMPLEMENTER Recherche la prochaine direction a suivre lorsque le point courant P
	 * est sur un bord du domaine mais pas dans un coin.
	 * Si la descente n'est pas finie, met la prochaine direction dans l'attribut this.direction  
	 * (c'est obligatoire d'utiliser cet attribut).
	 *<br>Selon la direction du gradient de f et si la descente n'est pas terminee, la prochaine direction
	 * peut etre vers l'interieur strict du domaine ou le long du bord dans un sens ou dans l'autre.
	 *<br> ATTENTION : si la methode de descente est finie, la methode doit passer this.finie a true.
	 * @see Couple#norme()
	 * @see Couple#produitScalaire(Couple)
	 * @see Couple#estPerpendiculaire(Couple)
	 * @see Couple#mult(double)
	 * @see Contrainte#getGradient()
	 * @see Contrainte#getBordUnitaire()
	 * @see Pb#gradientf(Couple)
	 * @see #seuil * 
	 * @param c la contrainte saturee par le point courant P
	 </font>*/
	private void directionSiBord(Contrainte c) {
		Couple dir = pb.gradientf(P);
		if(dir.norme()> seuil)
		{
			Couple gradcontraint = c.getGradient();
			if(dir.produitScalaire(gradcontraint)>0)//vers extrairieur
			{
				this.direction = dir.mult(-1);
			}
			else// vers intérieur ou sur un bord
			{
				Couple bordc = c.getBordUnitaire();
				if( dir.estPerpendiculaire(bordc))
				{
					this.direction = new Couple(0,0);
					this.finie = true;
				}
				else if(dir.produitScalaire(bordc)> 0)
					this.direction = bordc.mult(-1);
				else
					this.direction = bordc;
			}
			
		}
		else// norme trop faible, terminer le program
		{
			this.direction =new  Couple(0,0);
			this.finie = true;
		}
	}
	
	/**<font color="red">
	 * METHODE A IMPLEMENTER Recherche la prochaine direction a suivre lorsque le point courant P
	 * est dans un coin du domaine.
	* Si la descente n'est pas finie, met la prochaine direction dans l'attribut this.direction  
	 * (c'est obligatoire d'utiliser cet attribut).
	 *<br> ATTENTION : si la methode de descente est finie, la methode doit passer this.finie a true.
	 * <br> Voir le sujet sur http://www.infres.enst.fr/~hudry/optim/descente/TPDescente/ pour des explications.
	 * @see Couple#norme()
	 * @see Couple#produitScalaire(Couple)
	 * @see Couple#mult(double)
	 * @see Contrainte#getGradient()
	 * @see Contrainte#getBordUnitaire()
	 * @see Pb#gradientf(Couple)
	 * @param coin un tableau a deux cases pour les deux contraintes saturees par le point courant P
	 </font>*/
	private void directionSiCoin(Contrainte[] coin) {
		assert(coin.length == 2);
		Couple dir = pb.gradientf(P);
		Couple gradc0 = coin[0].getGradient().mult(-1);		
		Couple gradc1 = coin[1].getGradient().mult(-1);	
		Couple bord0 = coin[0].getBordUnitaire();
		Couple bord1 = coin[1].getBordUnitaire();
		Couple dcgrad = Couple.decompose(dir, gradc0, gradc1);
		Double p0 = dir.produitScalaire(bord0)/dir.norme();
		Double p1 = dir.produitScalaire(bord1)/dir.norme();
		
		if(dir.norme()> seuil)
		{		
			if(dir.produitScalaire(gradc0)<=seuil && dir.produitScalaire(gradc1)<=seuil)// le cas 1
			{
				this.direction = dir.mult(-1.0);
			}
			else if( dcgrad.x>= -1 * seuil && dcgrad.y >= -1 * seuil)// le cas 2
			{
				this.direction =new Couple(0,0);
				this.finie = true;
			}
			else{
				if(p0 <= p1) this.direction = bord0;
				else this.direction = bord1;
			}
		}
		else// norme trop faible, terminer le program
		{
			this.direction =new  Couple(0,0);
			this.finie = true;
		}
		
		
	}

	/**<font Color ="red">
	 * METHODE A IMPLEMENTER ; on considere une demi-droite parametree par t -> P0 + td (t >= 0) ;
	 * on pose g(t) = f(P0 + td) ; on suppose que l'on a g'(0) < 0 ; 
	 * on cherche un point P = P0 + td, t > 0, avec g'(t) > 0. 
	 * On rappelle que, par hypothese, la fonction f est convexe, ce qui entraine que g' est croissante.
	 * On pourra d'abord tester t = 1 puis, si necessaire, doubler la valeur de t successivement.
	 * @param P0 l'origine de la demi-droite.
	 * @param d la direction de la demi-droite.
	 * @return si on ne trouve pas de tel point avec t < Double.MAX_VALUE, la methode retourne -1 ;
	 *         <br>sinon la methode retourne une valeur de t avec g'(t) > 0.
	 </font>*/
	public double chercheSecondPoint(Couple P0, Couple d) {
		double t = 1;
		Couple P_actuel;
		double Gprim_actuel;
		
		P_actuel= P0.ajoute(d.mult(t));
		Gprim_actuel= pb.gradientf(P_actuel).produitScalaire(d);
		assert(Gprim_actuel < 0);
		
		while(Gprim_actuel < 0){
			t *=2;
			P_actuel= P0.ajoute(d.mult(t));
			Gprim_actuel= pb.gradientf(P_actuel).produitScalaire(d);
			
			if(t>= Double.MAX_VALUE)
				return -1;
		}
		return t;
	}
	
	/** <font Color ="red">
	 * METHODE A IMPLEMENTER ; on considere une demi-droite parametree par  t -> P0 + td (t >= 0) ;  
	 * on pose g(t) = f(P0 + td) ; on suppose que l'on a g'(0) < 0 et g'(t1) > 0 ; 
	 * on cherche un point P = P0 + t * d entre P0 et P0 + t1 * d avec g'(t) = 0. 
	 * Pour cela, on procede par dichotomie.
	 * <br> On peut utiliser la methode statique estNul de cette classe pour tester si une valeur 
	 * de type double est nulle ou non.
	 * @see Pb#gPrime(Couple, Couple, double)
	 * @see Descente#estNul
	 * @see Couple#ajoute(Couple)
	 * @param P0 l'origine de la demi-droite.
	 * @param d la direction de la demi-droite.
	 * @param t1 parametre tel que g'(t1) > 0.
	 * @return le point P P = P0 + t * d tel que g'(t) = 0. 
     </font>*/
	public Couple dichotomie(Couple P0, Couple d, double t1) {	
		// La valeur de retour est a  modifier
		// la derivee de  t -> P0 + td en t s'ecrit : pb.gPrime(P0, d, t);
		double t0 = 0;
		assert(pb.gPrime(P0, d, t0)<0);

		double t2 = (t0+t1)/2;
		while(!estNul(pb.gPrime(P0, d, t2)))
		{
			t2 = (t0+t1)/2;
			if(pb.gPrime(P0, d, t2)>0)
				t1 = t2;
			else
				t0 = t2;
		}
		Couple P = P0.ajoute(d.mult(t2));
		return P;
	}
	
	/**<font color="red">
	 * METHODE A IMPLEMENTER pour verifier la condition de (Karush) Kuhn et Tucker.
	 * Verifie qu'il s'agit bien d'un minimum en utilisant la condition de Kuhn et Tucker 
	 * @param P	Le point dans lequel on verifie qu'il s'agit d'un minimum.
	 * @see Domaine#estCoin(Contrainte, Contrainte)
	 * @see Domaine#estSurBord(Couple)
	 * @see Couple#norme()
	 * @see Couple#produitScalaire(Couple)
	 * @see Couple#estPerpendiculaire(Couple)
	 * @see Couple#decompose(Couple, Couple, Couple)
	 * @see Contrainte#getBordUnitaire()
	 * @see Contrainte#getGradient()
	 * @see Pb#gradientf(Couple)
	 * @return  	null si la condition de Kuhn et Tucker n'est pas verifiee
	 * 				<br>sinon
	 * 					<br>- le couple des multiplicateurs de Lagrange si on est sur un coin
	 * 					<br>- le couple forme par le multiplicateur de Lagrande et 0 si on est sur un bord
	 * 					<br>- (0, 0) si on est a l'interieur
	 * 
	 </font>*/
	public Couple KuhnTucker(Couple P) {

		Couple dir = pb.gradientf(P);
		if(domaine.estSurBord(P)==null)// cas 3: interieur, il faut egal à 0
		{
			if(estNul(dir.norme()))
				return new Couple(0,0);
			else
			{
				Couple mu = null;				
				return  mu;
			}
				
		}
		else if( domaine.estCoin(P) != null)// cas1: coin
		{
			Contrainte[] coin = domaine.estCoin(P);
			assert(coin.length == 2);
			Couple gradc0 = coin[0].getGradient();		
			Couple gradc1 = coin[1].getGradient();	
			Couple dcgrad = Couple.decompose(dir, gradc0, gradc1);
			if( dcgrad.x<= -1 * seuil && dcgrad.y <= -1 * seuil)// verifie la condition de Kuhn et Tucker
			{
				return dcgrad;			
			}
			else
			{
				Couple mu = null;				
				return  mu;
			}
		}
		else if(domaine.estSurBord(P)!= null){// cas 2

			ArrayList<Contrainte> conts = domaine.getContraintes();
			for(Contrainte c : conts)
			{
				Couple gradc = c.getGradient();
				Couple uc = c.getBordUnitaire();
				if(c.estSature(P)&&dir.estPerpendiculaire(uc))
				{
					double t = -1 * dir.norme()/gradc.norme();
					Couple coe_lag = new Couple(t, 0);
					return coe_lag;
				}
			}

			Couple mu = null;				
			return  mu;
		}
		else{//shall never happen
			assert(false);
			
			Couple mu = null;				
			return  mu;
			
		}
		
		
	}
	
	/**
	 * <br>Recherche la prochaine direction a suivre.
	 * La methode considere les cas ou :
	 *    		<br>P est a  l'interieur du domaine, 
	 *    		<br>P est sur un bord, 
	 *    		<br>P est sur un coin 
	 *<br> Si la descente est finie, la methode passe this.finie a true.
	 </font>*/
	public void prochaineDirection() {
		Contrainte c;
		Contrainte [] coin;
		c = this.domaine.estSurBord(P);
		if (c != null) {
			coin = this.domaine.estCoin(P);
			if (coin == null) {
				this.directionSiBord(c);
			}
			else {
				this.directionSiCoin(coin);
			}
		}
		else {
			this.directionSiInterieur();
		}
	}

	/**
	 * Rien a  modifier ; connaissant le point courant et la direction a  suivre, 
	 * la methode recherche le point courant suivant. 		
	 * @return  
	 * Si P0 est le point courant (qui peut etre a l'interieur de domaine, sur un 
	 * bord du domaine ou sur un coin) et si d est la direction a suivre, la methode retourne
	 *   <br>- soit le point courant suivant (qui peut etre a l'interieur du domaine, sur un bord, sur un coin)
	 *   <br>- soit null si elle a mis en evidence que la fonction f n'atteint pas de minimum 
	 * 		   sur le domaine considere.
	 */
	public Couple pointSuivantDescente() {
		double t1;
		/*
		 * Explications concernant l'instruction suivante.
		 * On considere une demi-droite parametree par  t -> this.P + t * this.direction ; 
		 * cette demi-droite peut partir de l'interieur du domaine, ou d'un bord, ou d'un coin 
		 * et peut longer un bord du domaine.
		 * Si la demi-droite est toute entiere dans le domaine, ce qui n'est possible que 
		 * si le domaine n'est pas borne,l'instruction retourne -1. 
		 * Sinon, elle retourne une valeur de t > 0 pour laquelle P + t * direction appartient 
		 * a un bord du domaine.		
		 */
		t1 = domaine.intersection(this.P, this.direction);
		// Si la demi-droite rencontre un bord du domaine et si la derivee de t -> P + t * direction
		// est positive en ce point d'intersection, on retourne ce point.
		if ((t1 > 0) && (pb.gPrime(this.P, this.direction, t1) <= 0)) 
			return this.P.ajoute(this.direction.mult(t1));
		
		// Si la demi-droite est toute entiere dans le domaine, on cherche un point ou la derivee de
		// t -> P + t * direction soit positive
		if (t1 < 0) {
			t1 = chercheSecondPoint(P, direction);
			// Dans le cas ci-dessous, le probleme n'atteint pas de minimum
			if (t1 < 0) return null;
			// sinon la derivee de t -> P + t * direction est positive pour t = t1
		}
		return dichotomie(P, direction, t1);
	}

	/**
	 * Permet de connaitre le probleme traite.
	 * @return le probleme traite
	 */
	public Pb getPb() {
		return this.pb;
	}

	/**
	 * Permet de connaitre le point courant de la methode de descente.
	 * @return le point courant
	 */
	public Couple getP() {
		return this.P;
	}

	/**
	 * Permet de connaitre la  direction a  suivre par la methode de descente a  partir du point courant P
	 * @return la direction a  suivre.
	 */
	public Couple getDirection() {
		return this.direction;
	}

	/** 
	 * permet d'initialiser le point de depart de la methode de descente
	 * @param p la valeur a  donner a  P.
	 */
	public void setP(Couple p) {
		this.P = p;
	}

	/**
	 * Permet de suspendre ou reprendre la methode de descente
	 * @param suspendre si le parametre vaut true, la methode est suspendue, elle est reprise 
	 * si le parametre vaut false.
	 */
	public void setSuspendre(boolean suspendre) {
		this.suspendre = suspendre;
	}

	/**
	 * Pour savoir si la methode de descente a ete stoppee
	 * @return la valeur de la variable booleenne stoppe
	 */
	public boolean isStoppee() {
		return this.stoppee;
	}

	/**
	 * Sert a  stopper la methode de descente
	 */
	public void stopper() {
		this.stoppee = true;
		this.finie = true;
	}

	/**
	 * Permet de savoir si la methode de descente est terminee
	 * @return true si la descente est terminee et false sinon
	 */
	public boolean isFinie() {
		return this.finie;
	}

	/**
	 * Apres que la descente soit terminee, sert a  savoir si le probleme traite
	 * atteint sa borne inferieure ou non
	 * @return true ou false selon que le probleme traite atteint sa borne inferieure ou non
	 */
	public boolean atteintBorneInferieure() {
		return this.atteintMinimum;
	}

	/** 
	 * Permet de preciser le test d'arret de la methode de descente 
	 * @param seuil si la norme du gradient est inferieur a  seuil, la methode de descente s'arrete
	 */
	public void setSeuil(double seuil) {
		this.seuil = seuil;
	}

	/**
	 * Permet de connaitre le nombre de pas effectues depuis le debut de la methode de descente
	 * @return le nombre de pas effectues depuis le debut de la methode de descente
	 */
	public int getNbPas() {
		return this.nbPas;
	}

	/**
	 * Permet de connaitre le temps d'attente entre deux pas de la methode de descente
	 * @return temps d'attente entre deux pas de la methode de descente, en millisecondes
	 */
	public int getDelai() {
		return delai;
	}

	/**
	 * Permet de fixer le temps d'attente entre deux pas de la methode de descente
	 * @param delai temps d'attente entre deux pas de la methode de descente, en millisecondes
	 */
	public void setDelai(int delai) {
		this.delai = delai;
	}
}
