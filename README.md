# MAGSC
***MAGSC: Globally Spatial Consistency-based Maximum Consensus for Efficient Point Cloud Registration***
![image](https://github.com/user-attachments/assets/43568ba6-6dac-462f-91b3-5df0fe76fac9)

## Requirements
- CMake
- PCL
- OpenMP

## Usage
The program is run with three input parameters:
1. an input file storing the source point cloud;
2. an input file storing the target point cloud;
3. resolution;

Example:
```
$ ./MAGSCReg ./data/src.ply ./data/tgt.ply 0.1
```
