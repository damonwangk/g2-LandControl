# g2-LandControl

基于 Agibot GDK / PNC 的 G2 底盘控制工具，提供键盘遥控、定时速度指令、软件停止，以及无地图和基于地图的两种 A→B 接口。

推荐工作方式：**Mac 维护代码并推送 GitHub，Ubuntu 同步代码并通过 SSH 控制 G2。Ubuntu 无需登录 GitHub，也无需安装机器人 GDK。**

| 能力 | 当前状态 |
|---|---|
| 通用底盘控制：键盘、速度指令、停止 | 已获现场使用者确认正常 |
| 无地图 A→B | 已实现速度段与相对位移换算；属于开环控制，实际点位待确认 |
| 雷达建图与地图导航 | 已封装厂家接口；建图效果和导航到点尚未实车验收 |
| 指定单轮舵角、无地图精确到点、全过程朝向闭环 | 当前未提供 |

详细操作见 [通用控制操作手册](通用控制操作手册.md)，建图与导航见 [MAPPING.md](MAPPING.md)。

## 1. 环境与控制链路

```text
Mac：GitHub 登录、维护和发布代码
  └─ SSH / rsync → Ubuntu：control_from_ubuntu.py
                    └─ ssh g2 → G2：Python 3.10 + Agibot GDK
                                       └─ PNC / DDS → 底盘
```

- **Ubuntu / Mac 启动端**：Python 3.7 或以上，OpenSSH；同步目录时使用 rsync 或 scp。已验证 Ubuntu Anaconda Python 3.7 与系统 Python 3.11。
- **G2 执行端**：机器人原生 Python 3.10、厂家提供的 `agibot_gdk` 和可用的 PNC 服务。本仓库不包含厂家 SDK。
- **SSH 别名**：Mac 的 `robot` 指向 Ubuntu；Ubuntu 的 `g2` 指向 G2。启动器使用 `BatchMode=yes`，因此 Ubuntu → G2 需要免交互认证。
- 启动器默认加载 G2 的 `/home/agi/app/env.sh /home/agi/app`，并调用 `/home/agi/Project_codex/g2-chassis-control/` 下的执行程序；更换部署路径时需同步修改 `control_from_ubuntu.py` 的 `WORKER`。

启动端只使用 Python 标准库，不需要额外安装 pip 依赖。**不要在 Ubuntu 直接给 `g2_chassis.py` 加 `--execute`**；实际控制统一通过启动器调用本体 SDK。

## 2. 首次部署：Mac → Ubuntu → G2

### 在 Mac 获取仓库

```bash
git clone https://github.com/damonwangk/g2-LandControl.git
cd g2-LandControl
```

已有本地工作副本可直接进入其目录，无需重复克隆。

### 从 Mac 同步到 Ubuntu

```bash
ssh robot 'mkdir -p "$HOME/底盘控制"'
rsync -av --exclude='.git/' --exclude='__pycache__/' ./ robot:~/底盘控制/
```

这一步只复制代码，Ubuntu 不参与 GitHub 认证。日常修改后也可以使用同一命令更新；它不删除 Ubuntu 上自定义的配置文件。

### 从 Ubuntu 同步执行程序到 G2

先登录 Ubuntu：

```bash
ssh robot
cd ~/底盘控制
```

然后复制四个执行端文件：

```bash
ssh g2 'mkdir -p /home/agi/Project_codex/g2-chassis-control'
scp g2_chassis.py ab_no_map.py ab_map.py mapping.py g2:/home/agi/Project_codex/g2-chassis-control/
```

**启动器不会自动上传程序。** Ubuntu 和 G2 的执行文件必须保持同版本；只更新 Ubuntu 可能继续运行 G2 上的旧代码。上面的文件同步不会启动 SDK 或运动。

## 3. 快速开始

下列命令均在 Ubuntu 的 `~/底盘控制` 内执行。

### 只预览

不加 `--execute` 时，只在启动端校验参数和输出 JSON，不连接机器人、不导入 GDK：

```bash
python3 control_from_ubuntu.py keyboard --speed 0.03 --turn-speed 0.10
python3 control_from_ubuntu.py velocity --vy 0.03 --seconds 2
```

### 键盘控制

加 `--execute` 才会申请控制并发送指令：

```bash
python3 control_from_ubuntu.py keyboard --speed 0.03 --turn-speed 0.10 --execute
```

`--speed 0.03` 为平移速度 3 cm/s；`--turn-speed 0.10` 为转动速度请求 0.10 rad/s，约 5.7°/s。

| 按键 | 动作请求 |
|---|---|
| W / S | 前进 / 后退 |
| A / D | 向左 / 向右平移 |
| Q / E | 左前 / 右前斜移 |
| Z / C | 左后 / 右后斜移 |
| J / L | 纯正 / 负转动请求，`vx=vy=0` |
| U / O | 前进并发送正 / 负 `wz` |
| N / M | 后退并发送正 / 负 `wz` |
| 空格 | 发送零速，留在键盘界面 |
| X | 尝试停止并退出 |
| Ctrl+C | 中断并尝试收尾 |

需要持续按住、由系统重复输入，或连续重复按键；不是按一下就持续行驶。最后一次有效更新约 0.35 秒后，若没有新输入，程序会发送零速。方向箭头不作为控制键。

### 独立速度指令

方向均以机器人自身坐标为准：正 `vx` 向前，正 `vy` 向左。未填写的速度分量默认为 0。

```bash
# 前进；后退改为 --vx -0.03
python3 control_from_ubuntu.py velocity --vx 0.03 --seconds 2 --execute

# 左右平移，不请求整车转动
python3 control_from_ubuntu.py velocity --vy 0.03 --seconds 2 --execute
python3 control_from_ubuntu.py velocity --vy -0.03 --seconds 2 --execute

# 左前斜移，合速度约 0.0424 m/s
python3 control_from_ubuntu.py velocity --vx 0.03 --vy 0.03 --seconds 2 --execute

# 前进并发送转动请求
python3 control_from_ubuntu.py velocity --vx 0.03 --wz 0.10 --seconds 2 --execute

# 软件停止
python3 control_from_ubuntu.py stop --execute
```

定时命令到时会尝试发送零速。`0.03 m/s × 2 s = 0.06 m` 是名义位移，不能替代真实位置反馈；SDK 初始化与申请控制不计入指令发送时长。

## 4. 参数含义与范围

| 参数 | 含义 | 当前程序范围 |
|---|---|---|
| `--vx` | 前后速度，m/s | 与 vy 的合速度 ≤ 0.10 m/s |
| `--vy` | 左右速度，m/s | 与 vx 的合速度 ≤ 0.10 m/s |
| `--wz` | 绕竖直轴的角速度请求，rad/s | 绝对值 ≤ 0.30 rad/s |
| `--seconds` | 单段速度指令时长 | 大于 0，最长 120 秒 |
| `--speed` | 键盘平移速度 | 默认 0.05 m/s |
| `--turn-speed` | 键盘转动速度请求 | 默认 0.10 rad/s |
| `--max-seconds` | 键盘会话时长上限 | 默认 3600 秒，最大 86400 秒 |
| `--handoff-task-id` | 明确交接已有远控任务 | 必须填写实际查询到的任务 ID |

这些是本程序的限制，不是硬件最大能力。默认速度发送频率为 50 Hz。

**`wz` 不是车轮转角。** 程序发送底盘速度，由底盘控制器分配车轮转向和驱动；当前没有单轮舵角控制入口。当前拒绝 `vy` 与 `wz` 同时非零；侧移时设 `wz=0`，转弯可用 `vx+wz`。`wz=0` 不包含朝向反馈纠偏，纯转动和转弯的具体轨迹也不能当作已校准角度。

## 5. 控制权与停止

PNC 空闲或已结束时可直接申请远控。若已有运行中的远控任务，新版会先退出并提示实际 ID，不重复申请。确认交接后，将提示中的数字代入 `ID`：

```bash
python3 control_from_ubuntu.py keyboard --speed 0.03 --turn-speed 0.10 --handoff-task-id ID --execute
```

程序只交接匹配 ID 的远控任务，不自动取消其他导航任务。结束后先尝试发零速，再仅按 ID 释放本次取得的任务。

同一账号的本套程序共用执行锁；已有实例运行时，独立 `stop --execute` 会通知它退出并由原进程收尾。这个锁不提供 Pad 或其他客户端之间的全局独占，软件停止也不是物理停稳的检测结果。

## 6. 两种 A→B

### 无地图：直接速度控制

```bash
# 参数示例，只预览；不是已确认的业务点位
python3 control_from_ubuntu.py --program no-map --dx 0 --dy 0.10 --speed 0.02
```

`ab_no_map.py` 支持显式 `vx/vy/wz/seconds` 速度段，也可将机器人当前坐标下的 `dx/dy` 与合速度换算成定时平移：`duration = hypot(dx,dy) / speed`。它不依赖地图，也没有位置闭环或自动避障。

### 基于地图：厂家导航

`ab_map.py` 将当前已定位位置作为 A，将配置中的地图坐标和最终 yaw 作为 B，调用厂家 `normal_navi()` 并监控任务状态。需要已有地图、在 Pad 完成重定位，并填写目标坐标。

```bash
# 查看未填点位的模板
python3 control_from_ubuntu.py --program map
```

地图导航的最终 yaw 不保证整段路径只做平移。建图、保存、切图、重定位和任务流程详见 [MAPPING.md](MAPPING.md)。

### 配置文件的位置

`configs/` 内是未填数值的模板，不能直接执行。启动器的 `--config` 路径会原样传到 G2，**不会自动上传 Ubuntu 的 JSON**。例如先复制自己填写的地图配置，再用 G2 绝对路径执行：

```bash
scp my_ab_map.json g2:/home/agi/Project_codex/g2-chassis-control/my_ab_map.json
python3 control_from_ubuntu.py --program map --config /home/agi/Project_codex/g2-chassis-control/my_ab_map.json --localized --execute
```

`--localized` 表示现场已完成 Pad 重定位，不是程序检测到了定位有效。`navigation_reported_success` 是 PNC 上报成功，实际精度仍需验收。

## 7. Python API

以下默认只演练接口，不导入 SDK：

```python
from g2_chassis import ChassisController, Velocity

with ChassisController() as chassis:
    chassis.run_velocity(Velocity(vy=0.03), duration=2.0)
```

实际调用须在本体 SDK 环境下显式 `execute=True`。持续控制可用 `update_velocity(vx, vy, wz)` 刷新请求，并周期调用 `tick()`；自行嵌入 API 时也应管理 `SessionLock`，避免与本套 CLI 并发。

## 8. 项目结构与离线测试

```text
control_from_ubuntu.py    # 启动端，SSH 调用 G2
g2_chassis.py             # 通用控制和速度 API
ab_no_map.py              # 无地图 A→B
ab_map.py                 # 地图目标导航
mapping.py                # SLAM / 地图工具
configs/                  # 未填点位的配置模板
tests/                    # 纯 Fake 回归
通用控制操作手册.md        # 按动作查命令
MAPPING.md                # 建图与导航流程
VERSION.json              # 文件校验与验证状态
```

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v
```

当前保留 45 项 Fake 测试，覆盖参数校验、控制时序、任务交接、停止与收尾、两种 A→B、预览以及模拟 SSH 的执行命令拼接。测试不连接真实 GDK 或机器人；通用控制的现场确认与地图导航等专项验收分开记录。

## 9. 常见问题

| 现象 | 排查方向 |
|---|---|
| 输出 `mode: preview`，没有动作 | 未加 `--execute`，属于正常预览 |
| `Task is not in IDLE, FAIL, SUCCESS or CANCELED state` | PNC 仍有未结束任务；确认执行端同版本，并按新版提示处理控制权 |
| 参数不认识或 G2 找不到程序 | 检查是否只更新了 Ubuntu，遗漏执行端文件同步 |
| Ubuntu 本地 JSON 存在，执行却找不到 | `--config` 执行路径属于 G2，需要先复制文件 |
| `No valid PTP device found` | 是 SDK 时间同步相关提示，需结合后续错误判断；不能仅凭此提示判定申请控制失败 |
| `commands_finished` | 指令流程完成，不代表通过位置反馈确认到点 |
