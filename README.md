# RedBuri
RedBuriの開発用リポジトリです．

## STM32
STM32 NUCLEO-F446REのmain.cにおいて，マージコンフリクトを防ぐため以下のファイルを作成してください．

`stm32/nucleo_f446re/Src/user_run.cpp`

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
