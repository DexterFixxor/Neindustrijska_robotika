# ROS: Neindustrijska robotika (FTN)

ROS paketi razvijani na vezbama iz predmeta *Neindustrijska robotika* na Mehatronici. 

## Potrebni programi
  * [VSCode](https://code.visualstudio.com/download)
  * [Docker](https://www.docker.com/)
  * [VcXsrv](https://sourceforge.net/projects/vcxsrv/) (za windows)
  * [GIT](https://git-scm.com/downloads)
  
## Dodaci za VSCode
Uputstvo za konfigurisanje dodataka se moze pronaci [ovde](https://code.visualstudio.com/docs/editor/extension-marketplace).

Obavezni:
  * CMake
  * CMakeTools
  * Dev Containers
  * Docker
  * ROS
  * Python

Opciono:
  * C/C++
  * C/C++ Extension Pack
  * Material Icon Themes

## Uputstvo za koriscenje

1. Klonirati repozitorijum preko <code>git clone</code> komande
2. Otvoriti folder u terminalu i pokrenuti komandu:  
<code>docker compose -f docker/docker-compose-dev.yml build</code> **(ukoliko ste na windows-u pokrenuti .yml fajl sa *windows* sufiksom)**
3. Prvo pokretanje moze da potraje i do 10 min
4. Nakon sto je preuzimanje i kompajliranje kontejnera gotovo, pokrenuti kontejner uz komandu:  
<code>docker compose -f docker/docker-compose-dev.yml up</code>
5. Odabirom **docker** ikonice sa leve strane (gde su i fajlovi, ekstenzije, itd), u gornjem levom uglu se moze videti kontejner pod nazivom: **ros_noetic:dev**. Ukoliko je sve instalirano kako treba, desnim klikom na njega dobijate ponudjenu opciju *Attach Visual Studio Code*.


**VAZNO**: Unutar kontejnera je moguce kreirati nove fajlove i foldere, medjutim sve sto je kreirano van /src foldera ce biti obrisano nakon sledeceg pokretanja procesa kompajliranja docker-compose fajla.  

# Test
Nakon sto je kontejner pokrenut i uspesno ste izvrsili *attach* na njega, mozete koristiti VSCode redovno kao i do sada. Otvaranjem tri terminala i pokretanjem sledecih komandi:
1. <code>roscore</code>
2. <code>rosrun turtlesim turtlesim_node</code>
3. <code>rosrun turtlesim_controler turtlesim_controler_node.py</code>

treba da se dobije *turtlesim* simulacioni prozor i da se robot pomeri na odgovarajucu tacku. Da bi treca komanda radila, prvobitno je potrebno pokrenuti *catkin build*, a zatim izvrsiti *source devel/setup.bash* komandu.

# Kratko uputstvo za GIT
Posto cemo kod razvijati na *dev* grani, a tek nakon sto proradi cemo ga spajati na *master* granu, potrebno je uraditi sledece:

<code>git pull origin dev</code> - ovo ce da povuce *dev* granu sa repozitorijuma na github-u.

Ukoliko zelite da imate svoju verziju projekta gde cete modifikovati stvari, savetujem da kreirate svoju granu koja ce se bazirati na *dev* grani. To se moze uraditi na sledeci nacin:
1. <code>git switch dev</code> da se prebacite na *dev* granu ako vec niste na njoj
2. <code>git branch NEKI_NAZIV</code> gde cete dati naziv grani u kojoj zelite da radite, ova grana ce biti identicna *dev* grani u trenutku pozivanja ove komande
3. <code>git switch NEKI_NAZIV</code> tako da mozete da radite u svojoj lokalnoj verziji koda bez da vas dotice to da li je nesto menjano na github-u

Ukoliko zelite da ispratite izmene na github-u, svaki put pre nego sto krenete da radite sa projektom, odradite sledece:
1. prebacivanje na *dev* granu:  
<code>git switch dev</code>
2. povlacenje izmena sa github-a na dev grani ukoliko ih ima (moguce je da ce neki delovi koda biti azurirani):  
<code>git pull origin dev</code>
3. vracanje na svoju granu:  
<code>git switch NEKI_NAZIV</code>
4. spajanje svoje grane sa novim izmenama iz *dev* grane (ovo nece raditi ukoliko ste menjali kod koji je vec postojao):  
<code>git merge dev</code>  
    4.1. ukoliko ste imali izmena u postojecem kodu, onda umesto <code>merge</code> koristiti <code>rebase</code> za tacku 4.