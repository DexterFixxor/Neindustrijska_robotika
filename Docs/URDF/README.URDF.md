# URDF: Universal Robot Description Format


Link ka detaljnijem objasnjenju:
[http://wiki.ros.org/urdf](http://wiki.ros.org/urdf)

URDF je XML fajl koji opisuje specifikaciju robota. Pored kinematske strukture moguce je i opisati dinamiku segmenata i zglobova robota. Glavni nedostatak je sto se ovaj zapis moze primeniti samo na stablo-konfiguracijama, odnosno na konfiguracijama koje nemaju zatvoren kinematski lanac. Opis pokriva:
- kinematski i dinamicki opis robota
- vizualnu reprezentaciju robota
- informacije o modelu kolizije robota

---

URDF se sastoji od 4 glavna dela:
1. Zaglavlje:   ```<?xml version="1.0"?>```
2. Glavni tag:  ```<robot></robot>```
3. Segment:     ```<link></link>```
4. Zglobovi:    ```<joint></joint>```

### Link ([wiki](http://wiki.ros.org/urdf/XML/link))

```xml
<link name="my_link">
    <inertial>
        <origin xyz="0 0 0.5" rpy="0 0 0"/>
        <mass value="1"/>
    <inertia ixx="100"  ixy="0"  ixz="0" iyy="100" iyz="0" izz="100" />
    </inertial>

    <visual>
        <origin xyz="0 0 0" rpy="0 0 0" />
        <geometry>
            <box size="1 1 1" />
        </geometry>
        <material name="Cyan">
            <color rgba="0 1.0 1.0 1.0"/>
        </material>
    </visual>

    <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
            <cylinder radius="1" length="0.5"/>
        </geometry>
    </collision>
</link>
```
Svaki zglob moze da se sastoji od vise vizualnih, kolizionih i inercionih elemenata.

### Joint ([wiki](http://wiki.ros.org/urdf/XML/joint))


```xml
 <joint name="my_joint" type="floating">
    <origin xyz="0 0 1" rpy="0 0 3.1416"/>
    <parent link="link1"/>
    <child link="link2"/>

    <calibration rising="0.0"/>
    <dynamics damping="0.0" friction="0.0"/>
    <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
    <safety_controller k_velocity="10" k_position="15" soft_lower_limit="-2.0" soft_upper_limit="0.5" />
 </joint>
```

Prilikom konstruisanja robota, potrebno je voditi racuna o koordinatnim sistemima. Tu vaznu ulogu igra ```<origin>``` tag.  
Origin predstavlja transformaciju od segmenta roditelja do segmenta deteta. I takodje je pozicioniran u originu **child** segmenta.

### Visual

Za vizualni i kolizioni deo segmenta moguce je pored prostih oblika koristiti i neke od 3D fajlova (STL, DAE), radi lepse i detaljnijeg prikaza.

Kako bismo ubacitli neki *mesh* u opis vizuala ili kolizije, treba u *geometry* opisu dodati:  

```xml
<mesh filename="package://$(find package_name)/putanja/do/fajla/fajl.stl>
```

Ukoliko se radi o prostom obliku, moguce je menjati njegovu boju:
```xml
<material name="naziv">
    <color rgba="0 0 0.8 1"/>
</material>
```