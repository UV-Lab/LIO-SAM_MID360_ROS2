# LIO-SAM_SC_LOC_MID360_ROS2

## Update

- 1.0 - 6.5 - format

- 1.1 - 6.5 - add openmp, sc loop

- 1.2 - 6.5 - add loc


## Tips

- WARN: 当使用Mid360时。可能因为点数过多导致处理滞后，修改 downsampleRate=2 可缓解问题
  - 直接体现为帧率下降，rviz显示卡顿，rqt: lidar:10 -> imageProj:9 -> featureExt:6

- TODO: add sc initLoc &  reLoc



## Related Package
  - [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)

  - [LIO-SAM_MID360_ROS2](https://github.com/UV-Lab/LIO-SAM_MID360_ROS2)
    - [LIO-SAM-MID360](https://github.com/nkymzsy/LIO-SAM-MID360)

  - [SC-LIO-SAM](https://github.com/gisbi-kim/SC-LIO-SAM)
    - [scancontext](https://github.com/gisbi-kim/scancontext)



  <!-- - [SC-LIO-SAM_based_relocalization](https://github.com/shallowlife/SC-LIO-SAM_based_relocalization) -->
