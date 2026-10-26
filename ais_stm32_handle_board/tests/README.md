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
| `native/handle_protocol_test.cpp` | `handle_protocol.cpp` | 指令の振り分け、未知IDで操作しないこと、テレメトリのID・長さ・バイト順 |
| `native/servo_test.cpp` | `handle_servo.cpp` | ONを1回だけ実行、UART処理中のOFF要求を次周期に保持、目標位置の反映 |

CANチップ・Dynamixel・RTOSの代替は `native/stubs/` にあります。
実機の通信タイミング、CRC、スタック余裕、サーボの機械的な動きは検証しません。

## 既存のホスト更新ツールとビルドスクリプトのテスト

```sh
python3 -m unittest discover -s ais_stm32_handle_board/tests -v
```

Python製のCAN更新ツール／プロトコルと、コンパイル失敗を返すビルドスクリプトを
確認します。vcanがない環境ではSocketCAN結合テストはスキップします。

## 今回の単体テストの範囲

4実行ファイル、14個の名前付きケースです。各ケースの関数名を実行結果に表示します。
C++の `assert` は `-UNDEBUG` により常に有効にします。追加のPython処理や
外部テストフレームワークのダウンロードは不要です。

- 共通サービス：基板別ID、バージョン分割、1/5/64文字の終端とゼロ埋め、
  不正文字・空・65文字・null、要求長、未知操作、更新拒否、ACK送信失敗。
- CAN transport：未読フレームの保持、ドライバの読み取り／送信失敗の伝達。
- サーボ：ONとOFFをそれぞれ一度だけ実行、処理中の次要求の保持、最新要求の優先、
  起動時ping失敗なら実行時UART操作をしないこと。
- ハンドルプロトコル：振動・サーボ・タッチ設定への指令振り分け、
  未知IDとファームウェア用IDで操作しないこと、4種類の応答データ形式。

タッチドライバは製品ヘッダを使い、テスト側のメソッド実装にリンクします。
CAN・Dynamixel・RTOSは `stubs/` の代替です。スレッドや実通信は使いません。
「通信テスト」「実機での復帰確認」は別途検討する対象であり、このスクリプトには含めません。
既存のPython更新ツール用テストも、このC++スクリプトからは実行しません。
