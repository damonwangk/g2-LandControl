# 雷达建图与地图导航

## 当前结论和依据

现有本体 SDK 快照具备完整的 **激光 SLAM → 保存地图 → 地图管理 → 重定位 → 地图目标导航** 接口，可以使用厂家现有链路实现地图导航，不需要先另建一套 ROS 导航栈。

已检查的本体配置包含 Livox 雷达及 SLAM/导航模块。通用底盘控制已获现场确认；建图质量、当前地图、A/B 点位及导航到点仍需现场核验。

接口依据为机器人随附的厂家 Python SLAM、Map、PNC 参考文档及对应 `gdk/{slam,map,pnc}.h` 头文件；本仓库不包含厂家 SDK。Python 的请求接口成功返回 `None`，失败抛异常；不能照搬 C++ 的返回码判断。

| 阶段 | 厂家 Python 接口 | 本次提供的入口 |
|---|---|---|
| 开始建图 | `Slam.start_mapping()` | `mapping.py record` |
| 停止并保存 | `Slam.stop_mapping()` | record 进程中按 Enter |
| 取消未保存建图 | `Slam.cancel_mapping()` | record 中断/输入关闭后尝试取消 |
| 列出地图/当前地图 | `Map.get_all_map()` / `get_curr_map()` | `mapping.py maps` |
| 切换地图 | `Map.switch_map(id)` | `mapping.py switch --map-id ID` |
| 当前位姿 | `Slam.get_curr_pose()` | 厂家 API；也可通过 Pad 记录点位 |
| 地图导航 | `Pnc.normal_navi(NaviReq)` | `ab_map.py` |
| 任务状态/取消 | `get_task_state()` / `cancel_task(id)` | 地图版任务监控与收尾 |

`Map.get_map(id)` 提供栅格地图及墙、不可通行区、引导点等结构；这里的交付目标是导航使用的地图，并未验证生成了可导出的三维场景模型。

## 后续实车建图流程

下面为后续建图流程，本次整理没有执行。

1. 确认雷达和 SLAM 服务可用，在 Pad 查看是否已有可用地图。若有合适地图，可直接选择并重定位，无需重复建图。
2. 加载本体 SDK 环境，在一个保持连接的交互终端启动 `mapping.py record --execute`。它调用 `start_mapping()` 后保持进程运行。
3. 用 Pad 或另一个终端的通用键盘控制让底盘低速覆盖作业区；观察地图是否连续，尽量经过有稳定环境特征的区域并形成回环。建图工具本身不发送底盘速度。
4. 在开始建图的终端按 Enter；调用 `stop_mapping()`，按厂家定义停止并保存。中断该流程会尝试 `cancel_mapping()`，不把取消当保存。
5. 用 `mapping.py maps --execute` 或 Pad 查看新地图。检查覆盖范围、障碍位置和重影，实际保存结果以列表和 Pad 为准。
6. 选择该地图并在 **G02 Pad 完成重定位**。切换地图只代表地图请求发送，不代表定位成功。
7. 在同一地图中确认 A、B 的位置和最终朝向。A 是导航开始时当前位姿；将 B 的地图坐标 `x/y`（米）、`yaw_rad`（弧度）以及地图 ID 填入配置。
8. 先预览，再在之后安排的实车测试中提交导航；检查路径、实际终点误差和最终朝向，再决定是否达到业务精度。

建图实际执行入口在本体：

```bash
source /home/agi/app/env.sh /home/agi/app
cd /home/agi/Project_codex/g2-chassis-control
# 这些是不连接 SDK 的预览
python3 mapping.py record
python3 mapping.py maps
python3 mapping.py switch --map-id 1
```

未来执行时给相应命令增加 `--execute`。`record` 需要交互终端，等待同一终端的 Enter；它与独立的底盘键盘控制进程配合使用，不作为无人值守自动建图任务。示例中的地图 ID 不是本任务已确认地图。

## 重定位为何先使用 Pad

厂家 `normal_navi()` 文档明确要求先在 G02 Pad 上重定位。虽然 SDK 也有 `Slam.global_local(grid_map_pt, map_pt)`，文档限制为 **仅支持在建图开始时的初始位姿重定位，其他位姿不能保证后续定位导航精度**。

因此本版不自动猜测起点、不从任意点调用 `global_local()`。`mapping.py localize` 会提示使用 Pad。地图导航的 `--localized` 只是记录现场已完成此步骤，不是程序检测到了定位有效。`loc_state/confidence` 的可靠取值和阈值尚未核验，代码没有编造定位状态枚举。

## 地图版 A→B 的执行过程

`ab_map.py` 验证配置 → 核对当前地图 ID → 要求 PNC 空闲或已结束 → 填写 `NaviReq.target` 地图坐标与 yaw 四元数 → 调用 `normal_navi()` → 关联新导航任务 ID → 轮询任务状态。

它不会自动切地图或替你重定位，也不会取消调用前已存在的任务。超时、中断或错误时，只对仍匹配的本次关联导航 ID 尝试取消。PNC 上报 SUCCESS 输出 `navigation_reported_success`，上报 FAIL/CANCELED 分别输出失败/取消；不会把无异常调用当作到点。

公开 Python `normal_navi()` 不返回任务 ID，`NaviReq` 也没有可自设的任务 ID。本版以“提交前空闲/结束，提交后读到新的普通导航 ID”关联任务；因此实际使用时须停止其他任务提交者。这不是跨客户端的独占锁。若请求发出后无法关联新 ID，会报告失败和 `navigation_request_sent=true`，并且不猜 ID 取消；这个未关联请求可能仍在运行，需要在 Pad 核对。

## 无地图与地图版如何选择

| 方式 | 需要什么 | 能提供什么 | 当前未保证 |
|---|---|---|---|
| 无地图速度段 | 明确 `vx/vy/wz/seconds` 或相对 dx/dy 与速度 | 直接底盘速度控制，不依赖地图 | 实际距离、闭环到点、自动避障、朝向纠偏 |
| 地图导航 | 可用地图、完成重定位、明确 B 坐标 | 厂家导航规划和任务状态 | 当前环境导航效果、精度、全过程固定朝向 |

对于你提出的“小范围侧移、方向不变”，无地图版可发送 `vx=0, vy=给定值, wz=0`。它只是请求固定方向平移；车轮转向准备、地面打滑、加减速会使速度乘时间偏离真实距离。

地图导航可以把 B 的最终 yaw 设为与 A 一致，但路径中仍可能转向。若后续要求厘米级到点、严格全过程固定朝向，需要先实测底盘与定位响应，再决定使用局部反馈控制还是厂家导航可配置的运动策略；本版没有用未经验证的里程计坐标或状态值冒充到点验证。
