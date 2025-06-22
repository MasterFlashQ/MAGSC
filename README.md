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
$ ./MAGSCReg src.ply tgt.ply 0.1
```
![配准](https://github.com/user-attachments/assets/1c5e5419-b6cd-47c1-895f-81ce434b2876)
