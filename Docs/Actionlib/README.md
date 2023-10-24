# Actionlib

U bilo kom sistemu baziranom na ROS-u, postoje situacije kada je potrebno poslati zahtev (request) čvoru da odradi određeni zadatak i da pošalje odgovor na taj zahtev (reply). Ovo se ostvaruje pomoću ROS [servisa](../Services/README.md).

U nekim situacijama servisu treba mnogo vremena da se izvrsi, a korisnik bi zeleo da ima mogucnost otkazivanja zahteva ili da dobija periodicne povratne informacije. Actionllib paket daje mogucnost da se kreira server koji izvrsava zadatke koji dugo traju i koji se mogu ranije otkazati.

ActionClient i ActionServer komuniciraju preko ROS Action protokola, koji je nadogradnja ROS poruka.

<center><img src="client_server_interaction.png"/></center>

Poruke akcija se sastoje iz sledecih elemenata:
* **CILJ**: zadatak koji treba da se izvrsi, npr PoseStamped poruka za kretanje baze robota
* **POVRATNA INFORMACIJA**: povratna informacija koja se salje klijentu periodicno o stanju, odnosno napretku, izvrsavanja zadatog cilja. Za kretanje baze bi to bila trenutna poza robota.
* **REZULTAT**: salje se klijentu po izvrsenju cilja. Za razliku od povratne informacije, ovo se salje samo jednom.



<!-- =========================================== -->
.action fajl
---
.action fajl se sastoji od definicije cilja, rezultata i definicije povratne informacije. Svaka definicija se razdvaja sa 3 crtice (---).

Fajlovi se smestaju u <code>/action</code> folder i izgledaju slicno kao <code>.srv</code> fajlovi.

Primer <code>/action/DoDishes.action</code> :
```python
#Define the goal
uint32 dishwasher_id
---
#Define the request
uint32 total_dishes_cleaned
---
#Define a feedback message
float32 percent_complete
```

## Build a package by Catkin

### Paket koji sadrzi *.action* fajl

Unutar CMakeList.txt je potrebno dodati pre <code>catkin_package()</code>:

```cmake
find_package(catkin REQUIRED genmsg actionlib_msgs)
add_action_files(DIRECTORY action FILES DoDishes.action)
generate_messages(DEPENDENCIES actionlib_msgs)
```
Takodje, u <code>package.xml</code> je potrebno dodati:

```xml
<depend>actionlib</depend>
<depend>actionlib_msgs</depend>
```

### Paket koji zavisi od actionlib API

CMakeList.txt:
```cmake
find_package(catkin REQUIRED genmsg actionlib_msgs actionlib)
add_action_files(DIRECTORY action FILES DoDishes.action)
generate_messages(DEPENDENCIES actionlib_msgs)
```

package.xml:
```xml
<build_depend>actionlib</build_depend>
<build_depend>actionlib_msgs</build_depend>
<exec_depend>actionlib</exec_depend>
<exec_depend>actionlib_msgs</exec_depend>
```

## Primer koda u C++ za ActionClient

```cpp
#include <chores/DoDishesAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<chores::DoDishesAction> Client;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "do_dishes_client");
    Client client("do_dishes", true); // true -> don't need ros::spin()
    client.waitForServer();
    chores::DoDishesGoal goal;
    // Fill in goal here
    client.sendGoal(goal);
    client.waitForResult(ros::Duration(5.0));
    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        printf("Yay! The dishes are now clean");
    printf("Current State: %s\n", client.getState().toString().c_str());
    return 0;
}
```
```yaml
Note: For the C++ SimpleActionClient, the waitForServer method will only work if a separate thread is servicing the client's callback queue. This requires passing in true for the spin_thread option of the client's constructor, running with a multi-threaded spinner, or using your own thread to service ROS callback queues. 
```

## Primer koda u Python za ActionClient

```python
#! /usr/bin/env python

import roslib
roslib.load_manifest('my_pkg_name')
import rospy
import actionlib

from chores.msg import DoDishesAction, DoDishesGoal

if __name__ == '__main__':
    rospy.init_node('do_dishes_client')
    client = actionlib.SimpleActionClient('do_dishes', DoDishesAction)
    client.wait_for_server()

    goal = DoDishesGoal()
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
```
  
## Action server C++

```cpp
#include <chores/DoDishesAction.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<chores::DoDishesAction> Server;

void execute(const chores::DoDishesGoalConstPtr& goal, Server* as)  // Note: "Action" is not appended to DoDishes here
{
  // Do lots of awesome groundbreaking robot stuff here
  as->setSucceeded();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "do_dishes_server");
    ros::NodeHandle n;
    Server server(n, "do_dishes", boost::bind(&execute, _1, &server), false);
    server.start();
    ros::spin();
    return 0;
}
```

## Action server Python

```python
#! /usr/bin/env python

import roslib
roslib.load_manifest('my_pkg_name')
import rospy
import actionlib

from chores.msg import DoDishesAction

class DoDishesServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('do_dishes', DoDishesAction, self.execute, False)
    self.server.start()

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    self.server.set_succeeded()


if __name__ == '__main__':
  rospy.init_node('do_dishes_server')
  server = DoDishesServer()
  rospy.spin()
```