# MAGSC
***MAGSC: Globally Spatial Consistency-based Maximum Consensus for Efficient Point Cloud Registration***
![技术路线](https://github.com/user-attachments/assets/6c499721-2334-4fb2-a0fb-9ca835260c7f)

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
