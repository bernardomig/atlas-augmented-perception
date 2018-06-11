# Interface

The interface exposes the Detection Service over HTTP/2 + gRPC for communication with the phone.

## Messages

The messages are typed, like in ROS, and are compiled to different languages. In our case, they are compiled to python for the server and csharp for the client (Unity). The following messages were designed:

### Namespace

For a greater professionality, all messages are in the package atlas.augmented.

### Geometry Messages

#### Size3 and Position3

```protobuf
syntax = "proto3";

package atlas.augmented;

message Size3 {
    float width  = 1;
    float heigth = 2;
    float depth  = 3;
}
```

```protobuf
syntax = "proto3";

package atlas.augmented;

message Position3 {
    float x = 1;
    float y = 2;
    float z = 3;
}
```

#### Bounding Boxes

```protobuf
syntax = "proto3";

package atlas.augmented;

import "Size3.proto";
import "Position3.proto";

message BoundingBox3 {
    Position3 position = 1;
    Size3     size     = 2;
}
```

### DetectedObject

```protobuf
syntax = "proto3";

package atlas.augmented;

import "BoundingBox3.proto";

message DetectedObject {
    // The unique id of the detected object
    uint64 id = 1;
    // The classification label
    string label = 2;
    // The tridimensional bounding box related to the object
    BoundingBox3 bbox = 3;
}
```

### The Detection Service interface

```protobuf
syntax = "proto3";

package atlas.augmented;

import "DetectedObject.proto";

service DetectionService {
    rpc GetDetectedObjects (DetectionServiceRequest) returns (stream DetectionServiceResponse) {} 
}

message DetectionServiceRequest {
}

message DetectionServiceResponse {
    repeated DetectedObject objects = 1;
}
```

## Build

To build the message python and csharp code, use the Makefile in the generated subdirectory. For example, to build the python code, execute the makefile command

    make gen-python

### Dependencies:

To install the dependencies, run the command `make get-deps`. For csharp you need nuget to be installed.
