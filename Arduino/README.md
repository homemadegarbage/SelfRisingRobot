# robo03

M5Atomで2自由度ロボットを制御し、学習済みポリシーで起き上がり動作を行うArduinoスケッチです。

## Files

- `robo03/robo03.ino` - M5Atom用の制御スケッチ
- `robo03/policy_network.h` - 学習済みポリシーをCヘッダ化したもの

`policy_network.h` は `robo03.ino` から読み込まれるため、必ず同じ `robo03` フォルダ内に置いてください。

## Required Libraries

Arduino IDEのライブラリマネージャなどで以下を入れてください。

- M5Atom
- Kalman
- ESP32Servo

ESP32ボード環境も必要です。

## Upload

Arduino IDEで `robo03/robo03.ino` を開き、M5Atom向けにビルドして書き込んでください。

## Wi-Fi Control

起動するとM5AtomがWi-Fiアクセスポイントを作成します。

- SSID: `robo1`
- Password: `password`
- URL: `http://192.168.42.1`

ブラウザからアクセスすると、サーボの手動調整、起き上がり開始/停止、自動起き上がりのON/OFFを操作できます。

## Button

M5Atom本体ボタンでも起き上がり動作を開始できます。

起き上がり中にボタンを押すと、起き上がり動作を停止して手動モードに戻ります。

## Notes

- サーボ1はGPIO 26、サーボ2はGPIO 32に接続する想定です。
- サーボのオフセットとパルス幅はWeb画面から調整でき、Preferencesに保存されます。
- `policy_network.h` は生成済みのポリシーネットワークです。別の学習結果を使う場合は、このファイルを差し替えてください。
