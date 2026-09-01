<!--
  Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>
  Shanghai Jiao Tong University, The Nezha Lab
  Key Laboratory of Polar Ecosystem and Climate Change
  State Key Laboratory of Submarine Geoscience
-->
# nezha_description

URDF/Xacro models and meshes for the Nezha robots. All multi-domain robots
follow one convention, with `nezha_mini` as the **reference model**.

## Directory layout

```
nezha_description/
├── common/                     # shared Xacro macros (domain-agnostic, no robot specifics)
│   └── UAV/
│       ├── component_snippets.xacro   # rotors_simulator component macros (Apache-2.0, ETH Zurich)
│       └── UAV_plugin.xacro           # Nezha transmedia rotor + motor-model macro
├── underwater_components/      # shared UUV sensor / snippet macros
├── <robot>/                    # one folder per robot (= its `namespace`)
│   ├── <robot>_base.xacro      # ENTRY POINT — properties, args, top-level assembly
│   ├── plugins.xacro           # robot-wide Gazebo plugins
│   ├── meshes/                 # robot meshes (<namespace>.dae / .STL, links, props)
│   ├── config/                 # controller / sensor yaml
│   ├── UAV/                    # aerial subsystem (rotors, body)
│   │   └── UAV.xacro
│   ├── UUV/                    # underwater subsystem (buoyancy, thrusters, sensors)
│   │   ├── UUV.xacro
│   │   ├── UUV_baselink.xacro
│   │   └── UUV_thruster.xacro
│   └── UGV/                    # ground subsystem (Husky only: wheels, decorations)
└── iris/, rexrov_spotlight/, spotlight/, nezha_gantry_*/   # single-domain / reference models
```

## The `namespace` convention

Every robot folder name **is** its Xacro `namespace`. Paths inside a model are
built from that argument so a model is self-locating and never hard-codes its
own folder name:

```xml
<xacro:arg name="namespace" default="nezha_mini"/>
<xacro:property name="namespace" value="$(arg namespace)"/>

<!-- meshes resolve under the robot's own folder -->
<xacro:property name="mesh_file"
  value="$(find nezha_description)/${namespace}/meshes/${namespace}2.dae"/>

<!-- subsystems are included relative to the namespace -->
<xacro:include filename="$(find nezha_description)/${namespace}/UAV/UAV.xacro"/>
<xacro:include filename="$(find nezha_description)/${namespace}/UUV/UUV.xacro"/>
```

**Shared** macros are included from the namespace-independent `common/` folder
instead, so they live in exactly one place:

```xml
<xacro:include filename="$(find nezha_description)/common/UAV/component_snippets.xacro"/>
<xacro:include filename="$(find nezha_description)/common/UAV/UAV_plugin.xacro"/>
```

## Include graph (nezha_mini)

```
nezha_mini_base.xacro          # properties + args (namespace, debug,
│                              #   enable_mavlink_interface, enable_motor_model)
├── plugins.xacro              # robot-wide Gazebo plugins
├── UAV/UAV.xacro
│   ├── common/UAV/component_snippets.xacro   (shared)
│   └── common/UAV/UAV_plugin.xacro           (shared)
└── UUV/UUV.xacro
    ├── uuv_sensor_ros_plugins/urdf/sensor_snippets.xacro   (upstream)
    ├── uuv_descriptions/urdf/common.urdf.xacro             (upstream)
    ├── UUV/UUV_baselink.xacro
    └── UUV/UUV_thruster.xacro
```

Sensor snippets come from the upstream `uuv_sensor_ros_plugins` package — robots
do **not** keep local copies of them.

## Adding a new robot

1. `cp -r nezha_mini <new_name>` and rename `nezha_mini_base.xacro` →
   `<new_name>_base.xacro`.
2. Set `<xacro:arg name="namespace" default="<new_name>"/>`. Because every
   internal path uses `${namespace}`, nothing else needs renaming.
3. Drop meshes into `<new_name>/meshes/` named `<new_name>.dae` / `<new_name>.STL`.
4. Tune the mass / geometry / motor `<xacro:property>` block at the top of the
   base file.
5. Reuse `common/UAV/*` and `underwater_components/*`; only add files for parts
   that genuinely differ from the reference model.

> Validate without launching Gazebo:
> `xacro <new_name>/<new_name>_base.xacro namespace:=<new_name> > /tmp/out.urdf`
