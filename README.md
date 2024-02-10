# Physics

## GLTF
- NuGetPackage で Microsoft.glTF.CPP をインストール
- min, max 関連でコンパイルエラーになる場合、windows.h より前に NOMINMAX を定義する
    ~~~
    #define NOMINMAX 
    #include <windows.h>
    ~~~
- リンカエラー 4099 が出る場合、(#pragma では回避できない) ので以下のようにする
    - Configuration Properties - Linker - CommandLine - AdditionalOptions - /ignore:4099
