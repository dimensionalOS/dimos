# Set up Acronym Dataset

Instructions are based off of https://github.com/NVlabs/acronym#using-the-full-acronym-dataset.

## Download Data

1. Download the acronym dataset and extract it to the `acronym/grasps` directory: [acronym.tar.gz](https://drive.google.com/file/d/1OjykLD9YmnFdfYpH2qO8yBo-I-22vKwu/view?usp=sharing)

2. Download the ShapeNetSem meshes (models-OBJ.zip) from [https://www.shapenet.org/](https://www.shapenet.org/) and
extract it to `acronym/models`.

The directory structure should look like this:

```
acronym
├── grasps
│   ├── *.h5
├── models
│   ├── *.obj
|   ├── *.mtl
```

## Get Manifold Waterproofing Library
In order to run physics simulation, we must waterproof the meshes.

Clone and build Manifold in the `acronym` directory.  This may take a while:

```
cd acronym
git clone --recursive git@github.com:hjwdzh/Manifold.git 
cd Manifold
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```
Note: We use a different `git clone` command than the Manifold repo.  The original repo's command 
hangs.

Additional information can be found at https://github.com/hjwdzh/Manifold.


The directory structure should look like this:
```
acronym
├── grasps
│   ├── *.h5
├── models
│   ├── *.obj
|   ├── *.mtl
├── Manifold 
```

## Waterproof the Meshes

Run the waterproofing script:

```
python3 tools/waterproof_meshes.py
```
This will waterproof the meshes in `acronym/models` and save them to 
`acronym/meshes`.  Additionally, a log file `acronym/failed.txt` will
document any meshes that failed to waterproof.

This may take a while.  After this is done, it is safe to
remove the `acronym/models` directory.

### Debugging

If a large number of files appear to be missing, try re-downloading the 
ShapeNetSem meshes.

Additionally, if your computer crashes, try reducing the number of cpu cores 
used in `waterproof_meshes.py`.

## Refactory Mesh Directory
A number of the scripts expect the mesh directory to include object categories 
in the file path.

e.g.

```
acronym 
├── meshes
│   ├── airplane
│   │   ├── id_1.obj
│   │   ├── id_2.obj
```

Run the following script to restructure the mesh directory:

```
python3 tools/refactor_mesh_dir.py
```

## Download Scene Data
Download `scene_contacts.zip` and `splits.zip` from [here](https://drive.google.com/drive/folders/1eeEXAISPaStZyjMX8BHR08cdQY4HF4s0?usp=sharing) and extract both to the `acronym` folder:

The directory structure should look like this (`models` and `Manifold` are 
optional):

```
acronym
├── grasps
├── models (optional)
├── Manifold (optional)
├── meshes 
├── scenes_contacts
├── splits 
```

## Final Notes
At this point, the `acronym` dataset should be ready for use.  
Additional instructions are also available for generating new scenes.  (see `README.md`)

