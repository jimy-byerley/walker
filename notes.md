 

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

- reducteurs planetaires composés: difficile a monter, difficilement reversible
- reducteurs a onde de deformation: facile a fabriquer, monter, reversible si bonne qualité d'impression en PETG

catalogue de roulements sur [123roulement](https://www.123roulement.com/roulement-palier/roulement-bille/simple-rangee)

## moteurs

il faut un contrôle en force, donc le moteur stepper est éliminé. le moteur a courant continu a  un couple qui dépend de la position rotor donc le joint sera faible

donc moteur synchrone. 

- les [nema ronds](https://www.omc-stepperonline.com/fr/24v-3000rpm-0-08nm-25w-1-80a-42x51mm-moteur-cc-sans-balais-42bya51-24-01) peuvent etre monté dans l'arbre creux
- [stepperonline](https://www.omc-stepperonline.com/fr/moteur-bldc) les fourni avec encodeur 3 positions pour l'asservir en connaissant la position des phases

## driver

- catalogue de drivers sur [simplefoc](https://docs.simplefoc.com/bldc_drivers)
- driver simple [stepperonline](https://www.omc-stepperonline.com/fr/controleur-de-moteur-cc-sans-balais-numerique-12v-48vdc-max-15-0a-400w-bld-510b)

## capteur de position

- capteur absolu sur arbre de sortie
- mesure optique sur un ruban imprimé placé sur la sortie reducteur voir dans le reducteur

- necessite un boitier noir

catalogue de capteurs sur [waveshare](https://www.waveshare.com/wiki/Main_Page)

### capteur intensité optique mono bande

- necessite plusieurs pistes separées

choix

- photo resistance

- [capteur luminosité](https://eu.robotshop.com/fr/products/capteur-lumiere-ambiante-numerique-haute-sensibilite-waveshare-tsl25911-i2c) I2C

### capteur intensité optique multi bande

- permet des pistes confondues (plus compact et plus simple a monter)
- permet une piste de reference de luminosité pour etre insensible a l'alimentation et vieillissement de la bande

choix

- [capteur RGB + LED](https://www.mouser.fr/ProductDetail/DFRobot/SEN0101?qs=Zcin8yvlhnPAaVRgPGvacA%3D%3D&mgh=1&vip=1) analog, voir [doc](https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/2107/SEN0101_Web.pdf)

- [capteur RGB + LED](https://www.amazon.com/Teyleten-Robot-TCS-34725-TCS34725-Recognition/dp/B087Z3K6P5/145-0111783-5754406?pd_rd_w=U3QIB&content-id=amzn1.sym.e56a2492-63c9-43e2-8ff2-0f40df559930&pf_rd_p=e56a2492-63c9-43e2-8ff2-0f40df559930&pf_rd_r=8XWYY363C57F0KZW0FMS&pd_rd_wg=bi5sm&pd_rd_r=2327b693-81b9-4b1a-899b-91aaed7cd432&pd_rd_i=B087Z3K6P5&psc=1) I2C

- [capteur spectral 8ch + LED](https://www.amazon.com/Waveshare-Precision-Compatible-Platforms-Including/dp/B08ZS7JKDD) I2C

- [capteur spectral 8ch + LED](https://www.amazon.com/Visible-Spectrum-Infrared-Spectrometer-arduino/dp/B0DBQKDV67/145-0111783-5754406?pd_rd_w=U3QIB&content-id=amzn1.sym.e56a2492-63c9-43e2-8ff2-0f40df559930&pf_rd_p=e56a2492-63c9-43e2-8ff2-0f40df559930&pf_rd_r=8XWYY363C57F0KZW0FMS&pd_rd_wg=bi5sm&pd_rd_r=2327b693-81b9-4b1a-899b-91aaed7cd432&pd_rd_i=B0DBQKDV67&psc=1) I2C

