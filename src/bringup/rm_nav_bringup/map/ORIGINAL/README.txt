这个目录放的是**原始地图（未被 P 图改过）**，只读用，不要直接拿去跑导航。

文件
----
RMUC2027_v2.ORIGINAL-20261031.pgm / .yaml
    · 来自 git 提交（10-31 02:15 那次建图的产物，slam_toolbox 2.6.10）
    · md5 = 2625b1d31241c0ad2514394c1189cdcc（与 map/RMUC2027_v2.pgm.bak-20261101-171056 一致）

怎么恢复原图
------------
    cd ~/HzMi_rmsimulation
    cp src/bringup/rm_nav_bringup/map/ORIGINAL/RMUC2027_v2.ORIGINAL-20261031.pgm \
       src/bringup/rm_nav_bringup/map/RMUC2027_v2.pgm
    cp src/bringup/rm_nav_bringup/map/ORIGINAL/RMUC2027_v2.ORIGINAL-20261031.yaml \
       src/bringup/rm_nav_bringup/map/RMUC2027_v2.yaml
    colcon build --symlink-install --packages-select rm_nav_bringup

怎么确认现在是哪一版
--------------------
    md5sum src/bringup/rm_nav_bringup/map/RMUC2027_v2.pgm
    # 2625b1d3... = 原图（没改过）
    # 其它值      = 已经被 P 过
    # 或者直接：
    python3 tools/scripts/mapping/map_edit.py stats src/bringup/rm_nav_bringup/map/RMUC2027_v2.pgm

P 图工具的自动备份（会越来越多个）
--------------------------------
每次 map_brush / map_edit 保存都会生成
    src/bringup/rm_nav_bringup/map/RMUC2027_v2.pgm.bak-<年月日-时分秒>
原图这一份是**额外的保险**，名字里带 ORIGINAL，不会被工具覆盖。
