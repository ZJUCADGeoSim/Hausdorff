# Efficient Hausdorff Distance Computation for Triangle Meshes with Error Bound

## What is it?

Giving two triangle meshes A.obj and B.obj, the following command
gives the lower bound L and upper bound U of Hausdorff distance from A.obj to
B.obj that U-L < error.

hausdorff -a A.obj -b B.obj -e error -t point

## Installation

This project can be built easily by:
```bash
git clone git@github.com:ZJUCADGeoSim/hausdorff.git
cmake -S . -B build
cmake --build build 
```

Then, you can find the binary `hausdorff` at `./build/bin`.

Run with our method for upper bound estimation:

```bash
./build/bin/hausdorff -a ./sample_data/hand-tri-smooth.obj -b ./sample_data/hand-tri.obj -t point
```

## More complicated usages

Run with Tang et al. [2009]'s method for upper bound estimation:
```bash
./build/bin/hausdorff -a ./sample_data/hand-tri-smooth.obj -b ./sample_data/hand-tri.obj -t triangle
```

Relative error can be applied via ...

## Motivation of this work

When working on the research about remeshing, we find that there is no
code to compute the Hausdorff distance reliably and efficiently.  So,
we made it.
