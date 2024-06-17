# LIO-SAM_SC_LOC_MID360_ROS2

## Update

- 1.0 - 6.5 - format

- 1.1 - 6.5 - add openmp, sc loop

- 1.2 - 6.5 - add loc

- 1.3 - 6.6 - fix loc extractCloud bug, mapOptmization:1047

- 1.4 - 6.13 - merge featureExtraction & imageProjection & mapOptimization，降低ros通信带来的性能损失

- 1.5 - 6.14 - 进一步优化节点合并

- 1.6 - 6.15 - 加入了缓存机制

- 1.7 - 6.17 - 加入SC ReLoc


## Tips

- [x] WARN: 当使用Mid360时。可能因为点数过多导致处理滞后，修改 downsampleRate=2 可缓解问题
  - 直接体现为帧率下降，rviz显示卡顿，rqt: lidar:10 -> imageProj:9 -> featureExt:6
  - 通过合并节点，现在已经解决该问题

- [ ] 合并节点引入了新的问题，存在多个类共同继承一个 ParamServer（必须绑定ROS2节点）

  - Publisher already registered for provided node name. If this is due to multiple nodes with the same name then all logs for that logger name will go out over the existing publisher

  - 可通过单例模式来解决，但会使所有变量都加上 ParamServer::get_instance() 前缀

- [ ] TODO: add sc initLoc &  reLoc



## Related Package
  - [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)

  - [LIO-SAM_MID360_ROS2](https://github.com/UV-Lab/LIO-SAM_MID360_ROS2)
    - [LIO-SAM-MID360](https://github.com/nkymzsy/LIO-SAM-MID360)

  - [SC-LIO-SAM](https://github.com/gisbi-kim/SC-LIO-SAM)
    - [scancontext](https://github.com/gisbi-kim/scancontext)



  <!-- - [SC-LIO-SAM_based_relocalization](https://github.com/shallowlife/SC-LIO-SAM_based_relocalization) -->
