# MAGSC
***MAGSC: Globally Spatial Consistency-based Maximum Consensus for Efficient Point Cloud Registration***

![技术路线](https://github.com/user-attachments/assets/edcdb969-4e4e-4774-bda7-9a2c26f8c38d)


## Requirements
- CMake>=3.5
- PCL>=1.8.1
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
