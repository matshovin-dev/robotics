tree ../../../.. -C -I '*.o'

└── robotics
    ├── check-includes.sh
    ├── experiments
    │   └── stewart-lab
    │       ├── build
    │       │   └── motion_patterns
    │       ├── Makefile
    │       └── src
    │           └── motion_patterns.c
    ├── libs
    │   └── math
    │       ├── build
    │       │   ├── test_geometry
    │       │   ├── test_matrix
    │       │   ├── test_utils
    │       │   └── test_vec3
    │       ├── DESIGN.md
    │       ├── include
    │       │   └── robotics
    │       │       └── math
    │       │           ├── geometry.h
    │       │           ├── matrix.h
    │       │           ├── utils.h
    │       │           └── vec3.h
    │       ├── Makefile
    │       ├── README.md
    │       ├── src
    │       │   ├── geometry.c
    │       │   ├── matrix.c
    │       │   ├── utils.c
    │       │   └── vec3.c
    │       ├── STRUCTURE.txt
    │       └── tests
    │           ├── test_geometry.c
    │           ├── test_matrix.c
    │           ├── test_utils.c
    │           └── test_vec3.c
    ├── platforms
    │   └── stewart
    │       ├── build
    │       │   └── test_kinematics
    │       ├── include
    │       │   └── stewart
    │       │       ├── geometry.h
    │       │       ├── kinematics.h
    │       │       └── pose.h
    │       ├── Makefile
    │       ├── src
    │       │   ├── forward.c
    │       │   ├── geometry.c
    │       │   ├── inverse.c
    │       │   └── pose.c
    │       └── tests
    │           └── test_kinematics.c
    ├── README.md
    └── viz-modules
        ├── common
        │   ├── include
        │   │   ├── udp.h
        │   │   └── viz_protocol.h
        │   └── src
        │       └── udp.c
        └── viz-stewart
            ├── build
            ├── Makefile
            ├── src
            │   └── main.c
            └── viz-stewart