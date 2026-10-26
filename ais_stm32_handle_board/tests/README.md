# Handle firmware tests

## 製品のC++モジュールを直接ビルドする確認

```sh
./ais_stm32_handle_board/tests/run_native_tests.sh
```

このスクリプトは次の製品コードをそのままg++でコンパイルし、C++テストとリンクします。
`.ino`の文字列抽出や、PythonへのC++埋め込みはありません。

| テスト | 製品コード | 確認内容 |
|---|---|---|
| `native/firmware_service_test.cpp` | 共通 `FirmwareService.cpp` | 2種類の基板ID設定、バージョン応答、更新拒否、送信失敗時にリセットしないこと |
| `native/can_transport_test.cpp` | `can_transport.cpp` | 未読ONを保持した受信、読み取り失敗の通知 |
| `native/servo_test.cpp` | `handle_servo.cpp` | ONを1回だけ実行、UART処理中のOFF要求を次周期に保持、目標位置の反映 |

CANチップ・Dynamixel・RTOSの代替は `native/stubs/` にあります。
実機の通信タイミング、CRC、スタック余裕、サーボの機械的な動きは検証しません。

## 既存のホスト更新ツールとビルドスクリプトのテスト

```sh
python3 -m unittest discover -s ais_stm32_handle_board/tests -v
```

Python製のCAN更新ツール／プロトコルと、コンパイル失敗を返すビルドスクリプトを
確認します。vcanがない環境ではSocketCAN結合テストはスキップします。
