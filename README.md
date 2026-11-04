# 基于安路 EG4S20 的音乐画屏——TD 源码工程

本工程对应已上板验证的最终版本 `arpeggio_melodies_20261031`。81 个 Verilog 文件及两份约束与最终版本构建输入逐字节一致。工程文件保留原有源码引用及编译顺序，改用相对工程路径，并保存最终版本的构建参数。

## 目录内容

| 路径 | 内容 |
| --- | --- |
| `lesson1_tf_hdmi_audio.al` | 安路 TD 原生工程，顶层模块 `top` |
| `src/` | FPGA 源码及所需官方 IP，包括加密 `.enc.v`、PLL、FIFO、SDRAM 包装文件 |
| `constraints/*.adc` | 管脚等物理约束 |
| `constraints/*.sdc` | 时钟及时序约束 |
| `tools/make_audio_corpus.py` | PC 端 WAV 转 PCM 素材工具 |
| `tools/music_sync/` | PC 端音乐拍点分析、试听校准及 CUE 导出工具 |
| `reference_build.json` | 最终上板版本的历史构建记录 |
| `SOURCE_MANIFEST.json` | 提交文件的 SHA256 校验清单，不包含清单自身 |

## 从 TD 图形界面构建

1. 使用安路 TD `6.2.168116`（安装版本名 `TD_6.2.1_Engineer_6.2.168.116`）。将工程解压到本地可写目录，建议使用不含空格及中文的路径。
2. 在 TD 中打开 `lesson1_tf_hdmi_audio.al`。确认器件为 `EG4S20BG256`，顶层为 `top`，源码和两份约束均已加载。
3. 运行综合 `syn_1`，完成后运行布局布线 `phy_1`。
4. 检查布局布线后的时序与布线报告。确认 setup、hold 无负裕量、无布线冲突后，再执行 Generate Bitstream / Bitgen。位流通常位于 `runs/phy_1/`，以 TD 实际显示的输出路径为准。

最终版本使用的参数已直接保存在 `.al` 的原生工程属性及运行策略中：

| 阶段 | 参数 | 设置 |
| --- | --- | --- |
| 综合 / Optimize Gate | `pack_effort` | `high` |
| 布局 / Optimize Placement | `seed` | `2` |
| 布线 / Optimize Routing | `fix_hold` | `on` |

保留上述策略即可从图形界面操作，无需另行运行 PowerShell 或 Tcl 构建脚本。可在 Process → Properties，或 Flow Runs 中右键运行项 → Run Properties → Flow Options 查看相关选项。部分内部参数（如布局种子）不在普通选项表中显示，以工程中已保存的设置为准。

本次打包已在临时副本中验证 TD 能打开工程、解析源码，并按其原生构建流程读取上述参数；没有重新执行综合、布局布线或生成位流。`reference_build.json` 中 setup `+0.111 ns`、hold `+0.019 ns` 为原最终版本结果，不是本次打包的新测试结果。相同源码和参数不等于已确认本次图形界面重建的位流逐字节相同，仍应以实际构建及时序检查为准。

## TF 卡素材

使用 FAT32，素材放在根目录，以 8.3 短文件名识别：

- 图片：`PIC1.BMP`～`PIC16.BMP`，24 位非压缩 BMP，宽不超过 1920、高不超过 1080；大图经轻量前置缩小后进入动态四 bank 缓存。
- 音乐：`SONG1.PCM`～`SONG16.PCM`，48000 Hz、单声道、16 位有符号小端、无文件头。
- 可选卡点表：`SONG1.CUE`～`SONG16.CUE`，与同编号 PCM 配对；没有有效点表时自动轮播采用定时间隔。

WAV 播放模式读取的是预转换的 PCM 数据，不能仅修改 WAV 文件后缀。工具用法见 `tools/WAV_PCM.txt` 和 `tools/music_sync/README.md`。WAV 转换工具使用 Python 3.9～3.12；音乐卡点工具入口 `run.cmd`、`export_cue.cmd` 使用 Python 3.12，首次运行需联网创建本地环境。它们用于制作 TF 卡素材，FPGA 工程构建不依赖这些工具。

源码包未附音视频素材、历史位流、仿真工程、构建数据库、日志或 Python 虚拟环境。所需器件原语库由上述 TD 安装提供。
