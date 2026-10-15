# 从另一台电脑通过 HTTP 控制 ROCOS

示例文件：[http_control.py](http_control.py)。使用 Python 3 标准库，无需安装 requests，也不需要安装 ROCOS、gRPC 或编译 C++。Windows 可将下面的 `python3` 替换为 `py -3`。

接口路径和 MoveJ 参数来自项目的 [ROCOS API.openapi.json](../ROCOS%20API.openapi.json)，鉴权细节按当前服务端实现处理。接口支持范围见 [REST_API_USAGE.md](../REST_API_USAGE.md)。

## 1. 连接地址

控制程序所在电脑应运行仿真并开放 HTTP 服务。例如在项目根目录启动：

```sh
./build/bin/rocosAppMain --sim=true --urdf=config/models/talon/robot.urdf --http_host=0.0.0.0 --http_port=8080
```

已有仿真在运行时无需重复启动。当前核对到控制电脑的局域网 IPv4 地址是 **192.168.1.117**；IP 改变后替换示例地址。

将 `http_control.py` 复制到另一台电脑，该电脑应能访问控制电脑的 TCP 8080 端口。在另一台电脑执行：

```sh
curl http://192.168.1.117:8080/api/robot/state
python3 http_control.py --base-url http://192.168.1.117:8080 state
```

另一台电脑不能把地址写成 `127.0.0.1`，那会连接它自己。连接超时先检查同网段连通性、防火墙和 HTTP 监听地址。示例直接访问控制器，不使用 `HTTP_PROXY` 等系统代理。

## 2. 先验证鉴权，不发送运动

```sh
python3 http_control.py --base-url http://192.168.1.117:8080 auth-test
```

测试向 MoveJ 发送空对象 `{}`：鉴权正确时仍因缺少 `joints` 被拒绝，因此不会执行运动，也不会修改模式或速度限制。

| 场景 | 预期业务码 | 解释 |
| --- | --- | --- |
| 不带 token | 3006 | 未获得控制权 |
| 无效 token | 3007 | token 不匹配 |
| 有效 token，但不带 client_id | 3007 | 客户端身份缺失 |
| 有效 token，但 client_id 不匹配 | 3007 | 客户端身份错误 |
| token 和 client_id 正确，MoveJ 参数为空 | 1001 | 鉴权通过，随后参数校验拒绝 |
| 持有控制权时，另一客户端申请 | 3008 | 单持有者冲突 |
| 释放后使用旧 token | 3007 | 已失效 |

每次请求使用新 TCP 连接也应通过鉴权，前提是两个请求头保持一致。成功示例不会使用调试 token 或强制抢占接口。

如果第一次申请控制权返回 3008，先在 HTTP viz 中释放控制权或退出持有控制权的客户端，等其令牌过期后再试。示例会报错退出，不抢占已有控制权。

## 3. 仿真运动示例

```sh
python3 http_control.py --base-url http://192.168.1.117:8080 move-demo
```

实际执行顺序：

1. 查询状态，确认是 HardwareSim、没有运动、关节已处于 CSP。
2. 申请控制权，保存服务端返回的 token 和 client_id。
3. 根据需要切换到 position 工作模式并使能。
4. 读取当前关节位置，关节 0 增加 `0.03 rad`（约 1.72°），其余关节保持原位。
5. 发送异步 MoveJ，通过受鉴权的 wait_move 等待完成，再核对实际位置。
6. 再发送 MoveJ 回到初始位置，停止，恢复原来的工作模式和使能状态，释放控制权。

指定关节和相对角度，例如关节 1 负向运动 0.02 rad：

```sh
python3 http_control.py --base-url http://192.168.1.117:8080 move-demo --joint 1 --delta -0.02 --velocity 0.1 --acceleration 0.3
```

`--joint` 从 0 开始。默认速度 `0.1 rad/s`，默认加速度 `0.3 rad/s²`；示例不传 `jerk`，不使用 single_axis/multi_axis 的 `max_vel` 参数，因此不会覆盖共享的速度、加速度、jerk 上限。不会切换 CSP/CSV/CST。

运动失败、Ctrl+C 或请求超时时，会尝试停止并释放控制权；失败分支不再发回位运动。网络中断时无法保证停止请求送达，脚本会输出清理失败原因。不要同时在其他 viz 中发送运动：当前 gRPC 接口不受 HTTP 单持有者控制权约束。

## 4. 鉴权要点与 OpenAPI 差异

当前实现要求：

```text
X-Rocos-Control-Token: 服务端返回的 data.token
X-Rocos-Client-Id: 服务端返回的 data.client_id
```

不是 `Authorization: Bearer ...`。只携带 token 会出现 3007，即使申请控制权成功也一样。

| 项目 | 当前实现 |
| --- | --- |
| 获取控制权 | POST /api/control/acquire；传 client_name 和 client_id |
| client_id | 不提供时服务端自动生成；后续必须使用返回的 data.client_id |
| token TTL | 默认 300 秒，可通过 --http_control_ttl_seconds 配置；响应中的 ttl_seconds 为准 |
| 连接绑定 | 校验 token 和 client_id；请求换 TCP 源端口不会导致失效 |
| 自动生成客户端 | OpenAPI 将 ControlToken 和 ClientId 声明为同时必需；两个请求头都要配置 |
| 续期 | POST /api/control/heartbeat 和通过鉴权的写请求会续期；所有普通 GET 不续期 |
| 过期/抢占 | 原 token 会失效；重新申请后应使用新 token，不能自动重发可能已执行的运动 |
| HTTP 返回状态 | HTTP 200 也可能 success=false，必须检查 JSON 的 success、code、message |
| 异步运动 | MoveJ 返回成功表示受理；wait_move 返回最终运动结果 |

Python 示例申请控制权后会启动独立心跳线程，按服务端建议的间隔续期（默认 5 秒）；release 前先停止并等待心跳线程退出。运动期间另用 wait_move 查询最终结果。不把 GET 状态轮询当成续期，也不使用 enable/stop 充当心跳。心跳收到 3006/3007 后停止，不会自动接管或重发运动。

仅心跳、不运动，保持超过原始 TTL 的验证命令：

```sh
python3 http_control.py --base-url http://192.168.1.117:8080 hold-test
```

该测试等待 `ttl_seconds + 10` 秒（默认 310 秒），验证原 token 仍有效，随后释放控制权。

`auth-test` 覆盖正常、缺失、错误、冲突及释放后的令牌。真实静默过期测试可运行：

```sh
python3 http_control.py --base-url http://192.168.1.117:8080 idle-test
```

`idle-test` 刻意关闭心跳，先申请控制权并验证鉴权，再保留凭据、不发送任何 HTTP 请求，等待 `ttl_seconds + 10` 秒（默认 310 秒）。随后验证旧 token 返回 3007，重新 acquire 获得新 token 后鉴权恢复，最后 release。整个过程仅发送空 MoveJ 来检验鉴权，不执行运动，不修改工作模式或运动限制。

静默期间的控制权可能被其他客户端在到期后重新申请，此时示例会报告 3008，不会强制抢占。仅保持网络连接或持续 GET 查询状态都不会续期。验证请求头和过期行为不等同于已经验证另一台电脑的网络环境。

## 5. 最简单的 curl 示例（Linux/macOS）

先查状态和申请控制权：

```sh
BASE=http://192.168.1.117:8080
CLIENT_ID="curl-$(python3 -c 'import uuid; print(uuid.uuid4())')"

curl -sS "$BASE/api/robot/state"

ACQUIRED=$(curl -sS -X POST "$BASE/api/control/acquire" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  -d "{\"client_name\":\"remote-curl\",\"client_id\":\"$CLIENT_ID\"}")

TOKEN=$(printf '%s' "$ACQUIRED" | python3 -c \
  'import json,sys; r=json.load(sys.stdin); assert r["success"] and r["code"]==0,r; print(r["data"]["token"])')
CLIENT_ID=$(printf '%s' "$ACQUIRED" | python3 -c \
  'import json,sys; r=json.load(sys.stdin); assert r["success"] and r["code"]==0,r; print(r["data"]["client_id"])')
```

申请失败时应停止后续步骤。使用两个请求头验证鉴权（空 MoveJ 不会运动，应返回 1001）：

```sh
curl -sS -X POST "$BASE/api/robot/movej" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  -d '{}'
```

仿真已使能并处于 position 模式时，可以运动到指定绝对关节位置。下面仅适用于 **7 轴且当前关节均为 0** 的仿真；其他初始位置用 Python 相对运动示例：

```sh
curl -sS -X POST "$BASE/api/robot/movej" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  -d '{"joints":[0.03,0,0,0,0,0,0],"velocity":0.1,"acceleration":0.3,"asynchronous":true}'

curl -sS -X POST "$BASE/api/robot/wait_move" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  -d '{"timeout":20}'
```

停止并释放控制权：

```sh
curl -sS -X POST "$BASE/api/robot/stop" \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID"

curl -sS -X POST "$BASE/api/control/release" \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID"
```

curl 示例的每一步都应检查响应 JSON；`curl` 进程正常退出或 HTTP 200 不代表业务成功。Python 示例已完成这些检查和最终清理。

curl 交互超过 TTL 时，也应定时 POST `/api/control/heartbeat`，同时携带两个请求头；单次 curl 调用不会启动后台心跳。

## 6. 本次仿真验证结果

2026-10-14，通过控制电脑的局域网地址 `http://192.168.1.117:8080` 实测：

- 上表中的七种鉴权场景全部符合预期，服务端返回 TTL 300 秒。
- 真实静默 310 秒、不发送任何 HTTP 请求后，旧 token 返回 `code=3007`、`message=control token expired`；重新 acquire 后，新 token 通过鉴权（空 MoveJ 返回参数错误 1001），最后成功释放控制权。
- `move-demo` 在 HardwareSim 上完成关节 0 的 0.03 rad 往返；去程测得 0.0299957155 rad，回程各关节位置均恢复为 0。
- 结束时 position 模式、使能状态保持原样，motion_busy=false、control_active=false，HTTP 控制权已释放。

验证从控制电脑本机连接其局域网地址完成；另一台电脑的防火墙、路由与网络连通性仍需按第 1 节自行验证。本次未重启或停止控制程序进程，也未修改共享运动上限。

## 7. 心跳升级验证（2026-10-15）

默认 TTL 仍为 300 秒。为快速覆盖相同的超时路径，以下测试在独立的测试服务上缩短 TTL，没有改变用户正在运行的仿真配置：

- 后端 `http_control_heartbeat_test` 使用 1 秒 TTL：仅心跳保持超过 TTL；错误身份、GET 查询不能续期；过期 token 无法复活；换 TCP 连接可正常续期；接管、释放后旧 token 失效。
- 真实浏览器双标签页使用独立仿真、6 秒 TTL：持有者超过 TTL 仍保持控制权，观察者不能普通申请；刷新页面恢复会话；通过界面显式接管后原持有者失权；主动断开立即释放；屏蔽心跳但继续 GET 时按时过期。心跳流程未发送机器人运动指令。
- Python 示例在独立仿真、1 秒 TTL 下运行 `auth-test`、`hold-test`、`idle-test`、`move-demo`，全部通过；仅心跳保持 11 秒仍有效，无心跳等待 11 秒后失效，运动往返后位置恢复。
- app 构建、4 项 CTest、viz 构建以及 6 项 `npm run test:control` 测试通过。

升级服务端和 viz 后，需要重新启动服务端并刷新或重启 viz，运行中的旧服务不会因源文件更新而自动支持心跳。HTTP 控制权目前不限制 gRPC 客户端。
