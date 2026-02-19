# MetaUrban + ROS2 Docker セットアップ

## 目的

Gazebo + ROS2 (Ubuntu PC) で動かしていたロボットシミュレーションを、**MetaUrban シミュレータに移行**する。
友達にも Docker コンテナとして配布し、任意の Ubuntu PC ですぐに起動できるようにする。

## 背景

- 元々 Gazebo でシミュを動かしていた（ROS2 Humble, Ubuntu）
- MetaUrban は都市環境のシミュレータで、歩行者・ロボット・自転車等のエージェントが組み込まれている
- MetaUrban には ROS2 ブリッジ (`bridges/ros_bridge/`) が既にあり、ZMQ 経由でセンサデータ（カメラ・LiDAR・BBox）を ROS2 トピックに流せる
- Mac での開発は描画（Panda3D + NVIDIA GPU）の問題があるため、**Ubuntu PC で動かす方針**に決定

## 今回追加したファイル

| ファイル | 説明 |
|---------|------|
| `Dockerfile.ros2` | MetaUrban + ROS2 Humble の Docker イメージ定義 |
| `docker-compose.yml` | GPU (NVIDIA) / X11 ディスプレイ転送 / ネットワーク設定 |
| `docker-entrypoint.sh` | コンテナ起動時に ROS2 環境を自動 source |
| `run.sh` | ワンコマンド起動スクリプト (`build` / `start` / `demo` / `ros`) |

## Ubuntu PC での前提条件

```bash
# Docker
sudo apt-get install -y docker.io docker-compose-plugin

# NVIDIA ドライバ確認
nvidia-smi

# NVIDIA Container Toolkit
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

# sudo なしで docker を使う
sudo usermod -aG docker $USER
# → 再ログイン
```

## 使い方

```bash
# 1. ビルド（初回のみ）
./run.sh build

# 2. アセットのダウンロード（初回のみ）
#    MetaUrban の 3D モデル等は別途ダウンロードが必要。
#    コンテナに入って pull_asset を実行する。
./run.sh start
# コンテナ内で:
python3 -m metaurban.pull_asset
# → フォーム (https://forms.office.com/r/tFBRFk7u4E) に記入し、コードを入力
# → アセットがダウンロード・展開される（約 4GB）
# → 完了したら exit でコンテナを抜ける
# ※ Docker ボリュームに保存されるので、次回以降は不要

# 3. デモで動作確認
./run.sh demo

# 4. ROS2 連携
./run.sh ros
# コンテナ内で tmux 等を使い2ターミナル:
#   ターミナル1: ros2 launch metaurban_example_bridge metaurban_example_bridge.launch.py
#   ターミナル2: cd /metaurban/bridges/ros_bridge && python3 ros_socket_server.py
```

## ROS2 ブリッジのアーキテクチャ

```
MetaUrban (Panda3D)                    ROS2 ノード
┌──────────────────┐                  ┌──────────────────┐
│ ros_socket_      │  ZMQ IPC         │ camera_bridge    │→ metaurban/image (sensor_msgs/Image)
│   server.py      │──────────────→  │ lidar_bridge     │→ metaurban/lidar (sensor_msgs/PointCloud2)
│                  │  ipc:///tmp/*    │ obj_bridge       │→ metaurban/object (vision_msgs/BoundingBox3DArray)
│                  │                  │                  │
│                  │←────────────── │ (cmd_vel sub)    │← /cmd_vel_mux/input/navi (geometry_msgs/Twist)
└──────────────────┘                  └──────────────────┘
```

## 次にやるべきこと (Ubuntu PC 側)

1. **Docker イメージのビルドテスト** — `./run.sh build` が通るか確認
2. **デモ起動テスト** — `./run.sh demo` で Panda3D の描画が表示されるか確認
3. **ROS2 ブリッジテスト** — `./run.sh ros` → 2ターミナルで起動し `ros2 topic list` でトピックが見えるか確認
4. **ビルドエラーの修正** — Dockerfile.ros2 は未テストなので、依存関係の不足やパス間違い等があれば修正する

## トラブルシューティング

### `Asset version file does not exist! Files: []`

アセット（3Dモデル等）がダウンロードされていない。`./run.sh demo` を実行する前に、コンテナ内でアセットをダウンロードする必要がある。

```bash
./run.sh start
# コンテナ内で:
python3 -m metaurban.pull_asset
```

`docker-compose.yml` で `metaurban-assets` ボリュームが `/metaurban/metaurban/assets` にマウントされており、初回はボリュームが空のため発生する。一度ダウンロードすればボリュームに保持される。

### `unzip: not found` (アセット展開時)

Dockerfile に `unzip` が含まれていなかった（修正済み）。再ビルドが必要：

```bash
./run.sh build
```

もし再ビルドせずに対処したい場合は、コンテナ内で直接インストール：

```bash
apt-get update && apt-get install -y unzip
```

### `containerd.io : 競合: containerd`（Docker インストール時）

Ubuntu 標準の `containerd` と Docker 公式リポジトリの `containerd.io` が競合している。どちらかに統一する：

```bash
# Docker 公式リポジトリ版を使う場合（推奨）:
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin

# Ubuntu 標準パッケージを使う場合:
sudo apt-get remove -y containerd.io
sudo apt-get install -y docker.io docker-compose-plugin
```

## 既知の注意点

- MetaUrban のアセット（3Dモデル）はイメージに含まれず、初回起動時に手動ダウンロードが必要（登録フォーム記入 + コード入力）
- `docker-compose.yml` は `network_mode: host` なのでコンテナ内の ROS2 トピックがホストからも見える
- conda は使わず、システム Python (3.10) に直接 pip install している（ROS2 との競合回避）
- 既存の `Dockerfile`（Anaconda ベース）は残してあるが、ROS2 非対応で ORCA 未コンパイルなので今回は使わない
