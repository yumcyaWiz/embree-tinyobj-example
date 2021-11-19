# embree-tinyobj-example

minimal raytracing example with a combination of embree and tinyobjloader

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