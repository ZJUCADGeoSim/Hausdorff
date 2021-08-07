# Economic Hausdorff

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

Run with Tang et al. [2009]'s method for upper bound estimation:
```bash
./build/bin/hausdorff -a ./sample_data/hand-tri-smooth.obj -b ./sample_data/hand-tri.obj -t triangle
```
