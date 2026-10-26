
## 最小修正の回帰テストを読む

まず次のC++ファイルの `main()` を読むと、操作と期待結果がわかります。

- `fixtures/can_receive.cpp`: OFF状態でON要求がRXB0に入っているところへ、
  目標位置のフレームを到着させます。未読のONが上書きされずに処理されることを確認します。
- `fixtures/servo_command.cpp`: ON要求1回が複数回実行されないことと、
  ONのUART処理中に届いたOFF要求が次の周期に処理されることを確認します。

`test_can_receive.py` / `test_regressions.py` は `.ino` から実際の処理を取り出します。
`firmware_test_support.py` はそれを `// FIRMWARE_UNDER_TEST` の位置に挿入し、
ホストのg++でコンパイルして実行する共通処理です。
ファームウェアの処理自体をテスト側へコピーして保守する構造ではありません。

CANとUART、RTOSは模擬しています。実機のタイミング、スタック残量、
サーボの物理動作が正常だと証明するテストではありません。

```sh
python3 -m unittest discover -s ais_stm32_handle_board/tests -v
```

`HANDLE_TEST_SOURCE` に旧版の `.ino` を指定すると、修正前の失敗を確認できます。
