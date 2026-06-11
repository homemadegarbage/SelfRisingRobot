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

## Notes

- `robo1_getup_ppo.zip`は`robo1.xml`と`robo1_env.py`の環境定義に依存しています。
- `assets/`内のSTLファイルがないとMuJoCoモデルを読み込めません。
- 実行はリポジトリ直下で行ってください。
