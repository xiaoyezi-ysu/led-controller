CubeMX 项目注意事项
====================

现象
----
直接编辑 .ioc 文件时，如果 CubeMX GUI 处于打开状态，
CubeMX 不会实时检测到文件的变更。
修改会等到 CubeMX 重新打开 .ioc 时生效。

解决方法
--------
1. 方案A — 修改前关闭 CubeMX GUI
2. 方案B — 直接修改 .ioc，然后用 CubeMX CLI 生成代码：
   ```
    & "CUBEMX_PATH.exe" -q gen.txt
    ```
    这样即使 CubeMX GUI 打开，CLI 也能读取最新的 .ioc
    并在下次 GUI 保存时覆盖 GUI 中的旧状态。

项目结构
--------
项目有两层同名目录：
```
stm32f103_CubemxCLI_demo0/           ← 外层 git root（无 CMakePresets.json）
  stm32f103_CubemxCLI_demo0/         ← 内层，实际项目（有 CMakePresets.json, .ioc）
```

**所有 CMake 命令必须在内层执行**，否则报找不到 CMakePresets.json。

Git 签出旧 commit 后编译烧录
---------------------------
**坑**：VSCode 的编译/烧录按钮可能仍使用上次构建的旧 .elf 文件。
签出旧 commit 后必须手动重新编译再烧录：

```
cd stm32f103_CubemxCLI_demo0      ← 进内层
cmake --build --preset Debug        ← 重新编译
openocd -s "SCRIPTS_PATH" -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program build/Debug/stm32f103_CubemxCLI_demo0.elf verify reset exit"
```

否则烧录的还是上次的旧固件。
