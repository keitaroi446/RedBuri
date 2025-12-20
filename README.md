# RedBuri
RedBuriの開発用リポジトリです．

## STM32
STM32 NUCLEO-F446REのmain.cppにおいて，マージコンフリクトを防ぐため以下の実行ヘッダーファイルを作成してください．

`stm32/nucleo_f446re/lib/run/run.hpp`

```cpp
#pragma once

namespace run
{
    inline void setup()
    {
        // 初期化
    }

    inline void loop()
    {
        // メインループ
    }
}