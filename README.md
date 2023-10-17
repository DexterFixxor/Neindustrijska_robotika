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

1. Klonirati repozitorijum preko *git clone* komande
2. Otvoriti folder u terminalu i pokrenuti komandu: *docker compose -f docker/docker-compose-dev.yml build* **(ukoliko ste na windows-u pokrenuti .yml fajl sa *windows* sufiksom)**
3. Prvo pokretanje moze da potraje i do 10 min
4. Nakon sto je preuzimanje i kompajliranje kontejnera gotovo, pokrenuti kontejner uz komandu: *docker compose -f docker/docker-compose-dev.yml up*
5. Odabirom **docker** ikonice sa leve strane (gde su i fajlovi, ekstenzije, itd), u gornjem levom uglu se moze videti kontejner pod nazivom: **ros_noetic:dev**. Ukoliko je sve instalirano kako treba, desnim klikom na njega dobijate ponudjenu opciju *Attach Visual Studio Code*.


# Test
Nakon sto je kontejner pokrenut i uspesno ste izvrsili *attach* na njega, mozete koristiti VSCode redovno kao i do sada. Otvaranjem tri terminala i pokretanjem sledecih komandi:
1. *roscore*
2. *rosrun turtlesim turtlesim_node*
3. *rosrun turtlesim_controler turtlesim_controler_node.py*

treba da se dobije *turtlesim* simulacioni prozor i da se robot pomeri na odgovarajucu tacku. Da bi treca komanda radila, prvobitno je potrebno pokrenuti *catkin build*, a zatim izvrsiti *source devel/setup.bash* komandu.