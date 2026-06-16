# robo1 getup PPO

MuJoCo上の2自由度ロボット `robo1` が倒れた姿勢から起き上がるための学習済みPPOモデルです。

## Files

このリポジトリを動かすために必要なファイルは以下です。

- `robo1_getup_ppo.zip` - Stable-Baselines3 PPOの学習済みモデル
- `robo1_env.py` - Gymnasium/MuJoCo環境
- `robo1.xml` - MuJoCoモデル定義
- `assets/` - `robo1.xml`が参照するSTLメッシュ
- `play_robo1_policy.py` - 学習済みモデルの再生
- `eval_robo1_policy.py` - 各倒立姿勢からの評価
- `search_all_getup.py` - 各倒れ姿勢ごとの起き上がり軌道候補を探索し、`getup_reference.py` に反映するための `BEST_SEQ` を出力
- `getup_reference.py` - 教師データ生成に使う、各倒れ姿勢ごとのサーボ目標角waypoint列
- `scripted_getup.py` - `getup_reference.py` に定義した起き上がり軌道をMuJoCo viewerで再生確認
- `pretrain_robo1_from_scripted.py` - `getup_reference.py` の軌道から観測と教師actionのペアを生成し、PPO方策を事前学習
- `train_robo1.py` - PPOによる追加学習
- `export_policy_header.py` - 学習済みモデルからArduino用Cヘッダを生成
- `requirements.txt` - Python依存パッケージ

## Setup

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

## Play

```powershell
python play_robo1_policy.py
```

特定の初期姿勢から再生する場合:

```powershell
python play_robo1_policy.py --pose roll_pos
python play_robo1_policy.py --pose roll_neg
python play_robo1_policy.py --pose pitch_pos
python play_robo1_policy.py --pose pitch_neg
```

## Evaluate

```powershell
python eval_robo1_policy.py
```

## Training

`getup_reference.py` に定義した起き上がり教師データから初期方策を事前学習します。

起き上がり軌道を再探索する場合は、以下を実行します。

```powershell
python search_all_getup.py
```
出力された `BEST_SEQ` または `REFERENCE_CANDIDATES` の配列を
`getup_reference.py` の `getup_sequence_for_pose()` に反映します。


教師データから初期方策を事前学習

```powershell
python pretrain_robo1_from_scripted.py
```

これにより `robo1_getup_ppo.zip` が生成されます。

続けてPPOで追加学習します。

```powershell
python train_robo1.py --model-in robo1_getup_ppo.zip --timesteps 200000 --n-envs 6 --model-out robo1_getup_ppo
```

ゼロからPPO学習する場合は `--model-in` を省略します。

```powershell
python train_robo1.py --timesteps 200000 --n-envs 6 --model-out robo1_getup_ppo
```

## Export for Arduino

学習済みモデルから `policy_network.h` を生成します。

```powershell
python export_policy_header.py robo1_getup_ppo.zip -o policy_network.h
```

生成した `policy_network.h` をArduinoスケッチ側に配置して使用します。

## Notes

- `robo1_getup_ppo.zip`は`robo1.xml`と`robo1_env.py`の環境定義に依存しています。
- `assets/`内のSTLファイルがないとMuJoCoモデルを読み込めません。
- 実行はリポジトリ直下で行ってください。
