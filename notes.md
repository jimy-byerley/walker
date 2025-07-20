 

[TOC]



# cinématique

## pied

### monobloc

la solution classique est de faire un simple bout de patte monobloc

- sphérique pour obtenir la meme adhérence dans toutes les directions
- excentré pour qu'il soit le premier agrippé par le sol

![schema]()

défaults

- adhérence limitée par la surface en contact
- tombe facilement dans les trous

Pour améliorer cela on fera des pieds passifs a plusieurs doigts

### plan rotulaire

- s'oriente tout seul a la normale du terrain
- quand l'effort du sol sors du cone, se fait replier sur le coté

![schema]()

### doigts à ressorts à l'avant

- tres similaire aux félins et canidés quand orientés vers l'avant
  - tres adapté pour la course
  - efficace uniquement pour les accelerations vers l'avant
- tres similaire aux insectes quand orientés vers les cotés
  - tres adapté au terrain convexe
  - efficace uniquement au maintien

![schema]()

### doigts a orientation planaire

L'idée est similaire au plan rotulaire mais sans ses défaults

![schema]()

- un mecanisme de parallelogrammes décale le placement de la rotule du lieu ou son mouvement est exploité

- 3 doigts est le min pour décrire un plan donc ne necessite pas de mise ne cohérence des doigts

- alternative à rotules

  ![schema]()

- alternative à pivots

  ![schema]()

## jambe

### 1 genoux (patte en V)

![schema]()

la solution classique, simple et efficace

### 2 genoux (patte en Z)

![schema]()

- meilleur rapport encombrement/foulée
- moins de debattement angulaire pour le pied
- probleme de jambe tendue et solution
- probleme de changement de configuration et compromis solution

## épaule

3 familles de solutions selon le compromis qui découle des directions d'efforts tolerées par les pivots et de la position des points de blocage de cardan de l'épaule (quand le pied est dans l'axe du premier pivot)

### épaule horizontale

![schema]()

la solution classique adoptée par Boston Dynamics et Unitree

adoptée par les mamifères (homoplate sur cage toracique)

- adapté pour la course: permet les plus grandes foulées dans le sens de la marche
- acceleration latérale tres limitée
- ne peut pas poser les pattes plus haut que l'épaule (blocage de cardan) ce qui limite la taille des obstacles enjambables

### épaule verticale

![schema]()

- holonome: meme acceleration et proprietés dans toutes les directions
- adapté pour porter des charges lourdes: pivots toujours orthogonaux a la direction de l'effort
- foulée limitée: les pied ne peuvent pas aller dessous
- la hauteur d'enjambée est limitée seulement par le pied
- ne peut pas courrir

### épaule oblique

![schema]()

intermédiaire entre les 2 premieres

- permet la course mais moins vite que *horizontale*
- porte plus lourd et plus stable que *horizontale*
- porte moins lourd que *verticale*
- peut enjamber des obstacles plus haut que *horizontale*

Les épaules avant et arrière peuvent également etre desymétrisées de manière a avoir des pattes arrières aptes à courrir et des pattes avant aptes à excalader

# motorisation

## réducteurs

les vérins du marché: lourds et lents

verins fait sur mesure: necessiterait moteur synchrone a arbre creux, pieces usinées, etc.

en general les vérins demandent beaucoup d'adaptation des pieces sur lequelles ils sont placés (encombrement du vérin, etc)

mieux vaut rester sur les moto-reducteurs rotatifs

voir les reducteurs elliptiques et planétaires composés

## moteurs

il faut un contrôle en force, donc le moteur stepper est éliminé.

le moteur a courant continu a  un couple qui dépend de la position rotor

donc moteur synchrone. 
