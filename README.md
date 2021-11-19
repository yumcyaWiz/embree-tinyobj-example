# embree-tinyobj-example

minimal raytracing example of combination of embree and tinyobjloader.

## Requirements

* [Embree3](https://github.com/embree/embree)
* OpenMP (Optional)

## Build

```
git submodule update --init
mkdir build
cd build
cmake ..
make
```

## Externals

* [tinyobjloader](https://github.com/tinyobjloader/tinyobjloader)