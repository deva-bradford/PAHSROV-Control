# bno055-rpi
AdafruitのArduino用BNO055ドライバ<https://github.com/adafruit/Adafruit_BNO055>をRaspberry Piに無理やり移植してみたものです。移植とか言ってますがAdafruit_Sensorなどは必要ありません。
Raspberry Pi用のC++で使えるライブラリがぱっと見みつからなかったため作りました。
なお、Experimentalと書かれている通り未実装の機能が多々あります。
自分の用途ではEulerのx軸さえ見れれば現状十分なのでこれからの修正は望めません。ぜひフォークでもしてくだしあ（

使い方はtest.cを見てください。ちなみに、delayを使っているというクッソしょうもない理由でwiringPiに依存しています。
```bash:test.cのコンパイル方法
g++ test.c  bno055.cpp -lwiringPi
```

使わせていただいたもの
jku - LSM9DS0
 <https://github.com/jku/LSM9DS0>
Adafruit - Adafruit_BNO055
 <https://github.com/adafruit/Adafruit_BNO055>
ghirlekar - bno055-python-i2c
 <https://github.com/ghirlekar/bno055-python-i2c>
