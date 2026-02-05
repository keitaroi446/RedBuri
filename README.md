# RedBuri
RedBuriの開発用リポジトリです．

## STM32 / NUCLEO_F446RE

実際の処理は `Src/main.c` ではなく `Src/user_run.cpp` に書きます。

`main.c` の `USER CODE` から以下を呼び出します。

- `setup_c()` : 初期化処理
- `loop_c()`  : メインループ処理

`Src/user_run.cpp` に以下を記述してください。

```cpp
namespace run 
{
    void setup()
    {

    }
    
    void loop()
    {

    }
}

extern "C" void setup_c()
{
    run::setup();
}

extern "C" void loop_c()
{
    run::loop();
}
```
