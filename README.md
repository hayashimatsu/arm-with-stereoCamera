# arm-with-stereoCamera

NHC12 六軸機械手臂 + Robotiq 2F-85 夾爪，腕上裝 Intel RealSense D455 立體相機。
在 Isaac Sim 裡用滑鼠拖曳一顆虛擬目標，手臂會即時用 IK（逆向運動學）跟隨；
也可以觸發相機拍照，取得 RGB + 深度資料。

A 6-axis robot arm (NHC12) with a Robotiq 2F-85 gripper and an Intel
RealSense D455 stereo camera on the wrist, running in NVIDIA Isaac Sim.
Drag a target in the viewport and the arm follows it in real time using
inverse kinematics (IK). The camera can also take snapshots (RGB + depth).

---

## 需要的資料夾 / Required folders

啟動這個 demo，最少只需要以下三個資料夾，其他都可以不要：

To run this demo you only need these three folders — everything else can
be left out:

| 資料夾 / Folder | 用途 / Purpose |
|---|---|
| `scenes/` | Isaac Sim 場景檔（.usd）。用 File > Open 打開它。 / The Isaac Sim scene file. Open it with File > Open. |
| `scripts/` | 執行時真正會用到的 Python 腳本（啟動、IK 控制、拍照）。 / The Python scripts the demo actually runs (start, IK control, capture). |
| `assets/` | 場景會用到的模型檔（機械手臂、夾爪、桌子、相機模組）。場景檔用相對路徑指到這裡，資料夾結構不能改。 / 3D model files the scene depends on (arm, gripper, table, camera). The scene references these by relative path, so keep the folder structure as-is. |

三個資料夾要放在同一層（同一個專案根目錄下），不能拆開。

Keep all three folders together under the same root directory. Do not
move or rename them separately.

---

## 需求 / Requirements

- NVIDIA Isaac Sim 6.0（或相容版本）
- 一台可以跑 Isaac Sim 的機器（GPU）

- NVIDIA Isaac Sim 6.0 (or a compatible version)
- A machine capable of running Isaac Sim (GPU required)

---

## 使用方式 / How to use

### 1. 開場景 / Open the scene

`File > Open` → 選 `scenes/arm310d_d455_ik_demo_r8.usd`

`File > Open` → select `scenes/arm310d_d455_ik_demo_r8.usd`

### 2. 啟動 / Start

`Window > Script Editor` → 用資料夾圖示開 `scripts/demo_start.py` → 按 `Ctrl+Enter`

這一步會自動：按 Play、載入 IK 控制腳本、啟動 IK 跟隨。
看到回傳 `"status": "ready"` 就表示成功。

`Window > Script Editor` → open `scripts/demo_start.py` → press `Ctrl+Enter`

This one step will: press Play, load the IK controller, and start IK
following. A result with `"status": "ready"` means it worked.

> 為什麼不要直接開 `ik_controller.py`？
> Isaac Sim 的 Script Editor「開檔案執行」和「互動 console」不會共用變數，
> 之後在 console 打指令會出現 `NameError`。`demo_start.py` 用特殊方式繞開這個問題，
> 所以一律從 `demo_start.py` 開始。
>
> Why not open `ik_controller.py` directly? Isaac Sim's Script Editor does
> not share variables between "run a file" and "interactive console" — you
> would get a `NameError` afterwards. `demo_start.py` works around this, so
> always start from `demo_start.py`.

### 3. 拖動目標 / Drag the target

在畫面裡用滑鼠拖 `/World/IKTarget`（綠色小方塊），手臂會自動跟著移動。

In the viewport, drag `/World/IKTarget` (the small green cube) with the
mouse. The arm follows it automatically.

拍照前建議先呼叫 `ik_follow_status()` 確認誤差夠小
(`last_error.pos_error_norm_m` 遠小於 `0.005`)。

Before capturing, call `ik_follow_status()` to check the tracking error is
small enough (`last_error.pos_error_norm_m` much less than `0.005`).

### 4. 拍照（選用）/ Capture a snapshot (optional)

```python
demo_capture()          # 立刻回 {"status": "scheduled"}，開始排程拍照
demo_capture_status()   # 過幾秒後呼叫，取得這次拍照的結果
```

成功後會在 `outputs/captures/<run_id>/` 產生 RGB、深度圖與 `result.json`
（這個資料夾是程式執行時自動建立，不需要事先準備）。

On success, files are written to `outputs/captures/<run_id>/`: RGB images,
depth data, and `result.json`. This folder is created automatically when
you run a capture — you don't need to create it yourself.

**已知限制 / Known limitation**：拍照功能可用，但深度數值尚未經過相機校正，
不能當作精確量測結果；`result.json` 裡的 `tabletop_objects` 只是「桌面上高
出來的區域」，不會辨識物體是什麼。IK 目標偶爾會被誤拍進畫面（機率不高但存在）。

The capture feature works, but the depth values are not camera-calibrated
— do not treat them as precise measurements. `tabletop_objects` in
`result.json` only detects "raised regions on the table", it does not
identify what the object is. There is also a known, occasional bug where
the IK target itself gets captured in a frame.

### 5. 停止 / Stop

```python
ik_follow_stop()   # 手臂回到記錄好的乾淨姿態 [0,0,0,0,0,0]
```

然後按時間軸的 ■ Stop。

Then press the ■ Stop button on the timeline.

---

## 可用函式 / Available functions

| 函式 / Function | 檔案 / File | 說明 / Description |
|---|---|---|
| `demo_start()` | `demo_start.py` | 一步啟動：Play + 載入腳本 + 啟動 IK / One-step start: Play + load scripts + start IK |
| `demo_stop()` | `demo_start.py` | 停止 IK 並回復乾淨姿態 / Stop IK and reset to a clean pose |
| `ik_follow_start()` | `ik_controller.py` | 啟動拖曳跟隨 / Start drag-and-follow |
| `ik_follow_status()` | `ik_controller.py` | 查詢目前追蹤誤差與狀態 / Check current tracking error and status |
| `ik_follow_stop()` | `ik_controller.py` | 停止並回復乾淨姿態 / Stop and reset to a clean pose |
| `demo_capture()` | `capture_d455.py` | 排程一次拍照 / Schedule one snapshot |
| `demo_capture_status()` | `capture_d455.py` | 取得上一次拍照的結果 / Get the result of the last snapshot |

---

## 這個 repo 不包含什麼 / What this repo does not include

這是從一個內部開發專案裡整理出來的「可執行版本」，只保留使用者實際跑 demo
需要的東西。開發過程的任務紀錄、審查紀錄、agent 協作文件、除錯用的舊版腳本
與一次性場景建構程式都沒有放進來。

This is a trimmed release from an internal development project — it keeps
only what's needed to actually run the demo. Development task logs,
reviews, agent-workflow files, old debugging scripts, and one-off
scene-build scripts are not included here.
