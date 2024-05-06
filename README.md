# Multi mid360 aligner



## Getting started


```
mkdir -p ws_Align/src
cd ws_Align/src
git clone https://github.com/SylarAnh/Multi_mid360_aligner.git
cd ../..
catkin_make 
```

## 开始标定
1. 修改launch文件中的雷达话题
2. 修改雷达初始角度，估算即可
3. 打开lidar驱动后，运行标定算法

```
roslaunch mm_align mm_lio_full.launch

```
4. 复制粘贴配准参数(/PCD/result.txt)到 mid360_three.yaml 中替换掉原有参数

