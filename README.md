# limx_ws

#### 介绍
通用机器人(逐际动力-TRON1)与通用人工智能(deepseek、Qwen等)的结合探索

#### 软件架构
```plaintext
limx_ws/
├── build/                                         # 编译生成的中间文件目录（自动生成，无需手动创建）
├── devel/                                         # 编译生成的可执行文件和库文件目录（自动生成，无需手动创建）
├── src/                                           # 源代码目录
│   ├── inspection_pkg/                            # 自定义巡检功能包（核心功能模块）
│   │   ├── CMakeLists.txt                         # CMake编译配置文件（必须）
│   │   ├── package.xml                            # ROS功能包描述文件（必须）
│   │   ├── scripts/                               # 可执行脚本目录（核心脚本存放）
│   │   │   ├── main_inspection.py                 # 主控制节点脚本（核心逻辑，建议命名明确）
│   │   │   ├── color_detection.py                 # 颜色检测模块脚本（复用/扩展）
│   │   │   ├── obstacle_avoidance.py              # 避障模块脚本（复用/扩展）
│   │   │   ├── black_line_detection.py            # 黑色线条检测脚本（病害检测）
│   │   │   ├── deepseek_agent.py                  # ds agent运行脚本（保留原有）
│   │   │   └── deepseek_prompt.py                 # ds prompt脚本（保留原有）
│   │   ├── motion_library/                        # 自定义动作库模块（运动控制核心）
│   │   │   ├── __init__.py                        # 包标识文件（必要）
│   │   │   ├── motion_controller.py               # 动作控制核心逻辑（必须）
│   │   │   └── utils.py                           # 通用工具函数（可选，如角度转换、参数校验）
│   │   ├── launch/                                # 启动文件目录（便捷启动配置）
│   │   │   ├── inspection.launch                  # 巡检功能启动文件（一键启动所有节点）
│   │   │   └── camera.launch                      # 相机驱动启动文件（可选）
│   │   └── config/                                # 配置文件目录（参数外部化）
│   │       ├── camera_params.yaml                 # 相机参数配置（校准参数等）
│   │       └── demo_params.yaml                   # demo专用参数（距离阈值、颜色列表等）
│   ├── robot-description/                         # 机器人描述文件仓库（URDF模型）
│   │   ├── urdf/                                  # URDF模型文件目录
│   │   │   └── tron1.urdf                         # TRON1机器人URDF模型文件（必须）
│   │   └── meshes/                                # 机器人模型网格文件目录（STL/DAE等格式）
│   │       ├── tron1_base.stl                     # 底盘网格文件（示例）
│   │       └── tron1_leg.stl                      # 腿部网格文件（示例）
│   └── robot-motion-control/                      # 机器人运动控制仓库（底层驱动）
│       ├── include/                               # 头文件目录（C++接口）
│       │   └── motion_control.h                   # 运动控制头文件（声明类/函数）
│       └── src/                                   # 源文件目录（C++实现）
│           └── motion_control.cpp                 # 运动控制底层实现（如电机驱动）
└── .catkin_workspace                              # ROS工作空间标识文件（自动生成）


#### 安装教程

1.  xxxx
2.  xxxx
3.  xxxx

#### 使用说明

1.  xxxx
2.  xxxx
3.  xxxx

#### 参与贡献

1.  Fork 本仓库
2.  新建 Feat_xxx 分支
3.  提交代码
4.  新建 Pull Request


#### 特技

1.  使用 Readme\_XXX.md 来支持不同的语言，例如 Readme\_en.md, Readme\_zh.md
2.  Gitee 官方博客 [blog.gitee.com](https://blog.gitee.com)
3.  你可以 [https://gitee.com/explore](https://gitee.com/explore) 这个地址来了解 Gitee 上的优秀开源项目
4.  [GVP](https://gitee.com/gvp) 全称是 Gitee 最有价值开源项目，是综合评定出的优秀开源项目
5.  Gitee 官方提供的使用手册 [https://gitee.com/help](https://gitee.com/help)
6.  Gitee 封面人物是一档用来展示 Gitee 会员风采的栏目 [https://gitee.com/gitee-stars/](https://gitee.com/gitee-stars/)
