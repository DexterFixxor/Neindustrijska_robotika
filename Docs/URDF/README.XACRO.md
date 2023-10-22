# Xacro: XML Macros

Upotreba makroa je veoma rasprostranjena u modelovanju robota preko URDF-a. Razlog tome je laka izmena parametara, definisanje konstanti i upotreba osnovnih racunskih operacija, kao i definisanje sablona za generisanje koda. 

Da bi bilo moguce koristiti XML makroe, potrebno je dodati sledece u zaglavlju: 

```xml
<robot name="naziv_robota" xmln:xacro="http://www.ros.org/wiki/xacro">

</robot>
```

Konstante
---
```xml
<xacro:property name="naziv_konstante" value="vrednost"/>
```
Blok koda
---
```xml
<xacro:macro name="default_origin">
    <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:macro>
```
Poziv ovakvog makroa:
```xml
<xacro:default_origin/>
```
---
Kao i kod makroa u programskom jeziku C, na mestu gde smo pozvali makro ce biti postavljen deo koda naveden u bloku definisanja makroa. Takodje, makroima je moguce proslediti parametre:

```xml
<xacro:macro name="default_inertial" params="mass">
    <inertial>
        <mass value="${mass}"/>
        <inertia ixx="1.0" ixy="0.0" ixz="0.0"
                 iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
</xacro:macro>
```

Pozivanje:
```xml
<xacro:default_inertial mass="10"/>
```
---
Prosledjivanje bloka parametara (blok sadrzi zvezdicu):
```xml
<xacro:macro name="blue_shape" params="name *shape">
    <link name="${name}">
        <visual>
            <geometry>
                <xacro:insert_block name="shape">
            </geometry>
            <material name="blue"/> <!-- ovo je definisano na drugom mestu -->
        </visual>
    </link>
</xacro:macro>
```

Prosledjivanje bloka parametara makrou:
```xml
<xacro:blue_shape name="base_link">
    <cylinder radius="0.42" length="0.01"> <!-- ovo je blok koji ce biti prosledjen -->
</xacro:blue_shape>
```
Prosledjivanje slozenog bloka (sa dve zvezdice):
```xml
<naziv_bloka>
    <color name="yellow"/>
    <mass>0.1</mass>
</naziv_bloka>
```
IF-ELSE
---
Primer ukoliko je neka promenljiva var jednaka stringu 'useit'
```xml
<xacro:if value="${var == 'useit'}"/>
```
```xml
<xacro:if value="${var.startswith('use') and var.endswith('it')}">
```

Argumenti
---
Prosledjivanje argumenata iz *launch* fajla:
```xml
<xacro:arg name="my_arg" default="false"/>
```
U launch fajlu:
```xml
<param name="robot_description" command="xacro $(arg model) my_arg:=true"/> <!-- model je XACRO fajl koji opisuje model robota-->
```

Include
---
Ucitavanje drugih XACRO fajlova:
```xml
<xacro:include filename="$(find naziv_paketa)/putanja/do/fajla/fajl.xacro"/>
<xacro:include filename="$(cwd)/other_file.xacro" />
<!-- cwd je za current work dir, odnosno relativna putanja u odnosu na trenutni fajl u kome se ova komanda poziva-->
```

Namespace
---
Ukoliko postoje parametri sa istim nazivima, kako bismo izbegli nesuglasice i probleme, mozemo koristiti namespace:
```xml
<xacro:include filename="file.xacro" ns="namespace"/>

${namespace.property} <!--Na ovaj nacin se pristupa stvarima iz file.xacro-->
```

YAML
---
Parametri (properties) mogu da budu liste ili recnici - deklarisani preko python sintakse.
```xml
<xacro:property name="props" value="${dict(a=1, b=2, c=3)}"/>
<xacro:property name="props_alt" value="${dict([('1a',1), ('2b',2), ('3c',3)])}"/>
<xacro:property name="numbers" value="${[1,2,3,4]}"/>
```

Takodje je moguce iscitati parametre iz YAML fajla:
```xml
<xacro:property name="yaml_file" value="$(find package)/config/props.yaml" />
<xacro:property name="props" value="${load_yaml(yaml_file)}"/>
```

Ako je YAML fajl dat kao:
```yaml
var1: 10
var2: 20
link1:
    name: "naziv"
```
Pristup iz YAML fajla:
```xml
<xacro:property name="val1" value="${props['val1']}" />
<xacro:property name="link1_naziv" value="${props['link1']['name']}" />
```
