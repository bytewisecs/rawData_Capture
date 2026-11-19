# mmWave radar raw ADC 采集



```text
AeroSense_raw_capture/
├── datacapture/                # 原始 ADC 采集代码
│   ├── cfg/radar/             # IWR1843 的 TI CLI 配置
│   ├── cfg/dca1000/           # DCA1000 参数、IP 和串口
│   ├── drivers/dca1000.py     # 原 DCA1000 控制库副本
│   ├── third_parts/fpga_udp/  # 原项目第三方 C++ 接收库，保留源码和许可证
│   ├── hardware.py           # 串口、DCA 启停
│   ├── native_udp.py         # 调用 fpga_udp.read_data_udp()
│   ├── udp.py                # 字节计数定位、包重组、丢包标记
│   ├── decode.py             # LVDS -> complex64 ADC cube
│   ├── capture.py            # 采集线程与独立写盘线程
│   └── recording.py          # 流式 NPY、逐帧哈希和完整性索引
├── dataSet/                  # adc_*.npy、同名 .json 和 .frames.jsonl
├── test.py                   # 读取 NPY，完整性检查，Range/Doppler FFT
├── validation/               # FFT 检查报告及 fft_results.npz（运行测试时创建）
├── tests/                    # 采集模块的针对性检查
├── wheelhouse/               # Jetson Linux aarch64 / Python 3.8 离线依赖
├── .venv/                    # 本目录专用环境（setup.sh 创建）
├── run_capture.sh
└── setup.sh
```

## 运行

```bash

# 首次安装；已安装后不用每次运行
./setup.sh

# 检查配置、第三方库、串口权限、UDP 缓冲和空闲端口，不启动雷达
./run_capture.sh --check

# 采集标称 30 秒，自动保存到 dataSet/
./run_capture.sh --duration 30

# 自定义输出文件，禁止覆盖已有文件
./run_capture.sh --duration 30 --output dataSet/view_001.npy

# 或者明确采集帧数
./run_capture.sh --frames 100 --output dataSet/short_capture.npy

# 读取最近的 NPY，检查所有帧，并对所有帧计算 FFT
.venv/bin/python test.py

# 指定 NPY；--fft-frames 20 仅抽取 20 帧做 FFT，完整性仍检查全部帧
.venv/bin/python test.py dataSet/view_001.npy --fft-frames 20
```

默认硬件参数来自 `datacapture/cfg/dca1000/dca1000.json`：

主机 `192.168.1.132`，DCA1000 `192.168.1.150`，（需要根据实际硬件修改IP）

数据端口 `4098`，控制端口 `4096`，雷达 CLI `/dev/ttyACM0`。
JSON 中 `RadarConfig.radarCFG` 相对于该 JSON 文件解析，默认使用本目录内的 `profile_AeroSense.cfg`。
也可通过 `--radar-config`、`--dca-config`、`--host-ip`、`--dca-ip`、`--cli-port` 覆盖。
IP 参数不改写网卡或 FPGA 的 Ethernet 配置。

默认雷达周期 50 ms（20 Hz）。`--duration 30` 换算为 600 帧，按帧数结束；
配置、启停和最后写盘会产生额外的墙钟时间。
Ctrl+C / SIGTERM 会停止雷达并将已写入帧整理成可读取的短 NPY，同时标记本次采集不完整。
突然断电或 SIGKILL 留下 `.npy.part`，不会把它标为成功的 NPY。

## 数据约定

```python
import numpy as np
adc = np.load("dataSet/view_001.npy", mmap_mode="r", allow_pickle=False)
# dtype: complex64
# shape: [frame, loop, tx_slot, rx_slot, adc_sample]
# 默认 30 秒：[600, 32, 3, 4, 256]
frame = adc[0]  # [32, 3, 4, 256]
```

这些是 FFT、窗函数和去直流之前的原始 ADC I+jQ，int16 I/Q 转 complex64 后数值不缩放、不量化。
TX 槽位按原 CFG 的物理顺序 `[0, 2, 1]` 保存，RX `[0, 1, 2, 3]`。
原项目 SDK 的两条 LVDS lane 使用 `[Q0,Q1,I0,I1]`，默认 `--iq-order q_first`。
只有固件确实使用 I-first 时才指定 `--iq-order i_first`。
一帧约 768 KiB，600 帧的 NPY payload 为 450 MiB。

同名 `.frames.jsonl` 逐行记录 NPY 行号、设备流帧号、估计时间、完整性和 ADC 字节 SHA256。
同名 `.json` 保存实际 shape、完整 CFG 内容、配置参数、收包统计、丢失字节数和整个 payload 的 SHA256。
丢包的帧保留槽位并补零，明确标记 invalid，程序返回非零，不静默删除帧。
流末尾不属于请求帧数的未完成数据不写入 NPY，可从 diagnostics 查看。
时间来自主机启动命令和帧计数估算，不是硬件曝光时间。

## test.py 的 FFT 和验证

Range FFT：每条 chirp 沿最后一维去均值、乘 Hamming 窗、做 FFT。
Doppler FFT：同一 TX/RX 沿 loop 维乘 Hamming 窗、FFT、fftshift，保留零多普勒能量。
不把不同 TX 的 chirp 当作连续的慢时间样本。
慢时间采样间隔为 `TX 数量 × chirp_period_s`；多通道功率平均生成 `[range, doppler]` 图。

测试读取全部帧，核对 shape、int16 I/Q 范围、有限值、哈希、帧连续性和采集完整性。
还用已知正/负 Doppler 合成信号验证轴和峰值，并把真实帧的 FFT 与直接 DFT 比较。
它报告 ADC 饱和比例、各通道 RMS、距离/速度分辨率和平均频谱峰值。

输出 `validation/<npy文件名>/report.json` 和 `fft_results.npz`。
NPZ 包含首帧的复数 Range FFT、复数 Doppler FFT、平均 Range-Doppler 功率、平均距离功率，以及距离/频率/速度轴。
通过这些检查说明传输、保存和计算一致；实际距离偏差、速度正负方向和目标位置精度需要已知位置/速度的目标作标定。

## 接收库和环境

调用链：`run_capture.sh -> datacapture.cli -> capture -> DcaLiveSource -> FpgaUdpReceiver -> fpga_udp.read_data_udp -> DcaPacketAssembler -> decode_lvds_frame -> NpyRecorder`。
数据 UDP 由复制的第三方 C++ 库接收，`udp.py` 只负责组帧。
DCA 控制命令沿用 `drivers/dca1000.py` 的原实现。
`source_manifest.json` 记录复制源和 SHA256，第三方源码和许可证原样保留。

`setup.sh` 使用当前目录的离线 wheel，创建专用 `.venv` 并从当前目录第三方源码编译 `fpga_udp`。
复制整个目录后仍需保留 Python 3.8、编译器及开发头文件等系统依赖；不同架构需准备对应 wheel。
没有通过 PYTHONPATH、符号链接或 editable install 引用旧项目。

UDP 缓冲预检查要求 `net.core.rmem_max >= 16777216`。
已有持久配置为 `/etc/sysctl.d/99-aerosense-udp.conf`，如运行值不符可重新加载：

```bash
sudo /lib/systemd/systemd-sysctl --prefix=/net/core/rmem_max
```

## FFT 图片

`test.py` 默认在 `validation/<数据文件名>/` 中同时输出：

- `range_fft.png`：所有选中帧、TX/RX 平均后的距离功率谱。
- `doppler_fft.png`：Range-Doppler 线性功率沿距离求和后的多普勒谱。
- `range_doppler.png`：平均 Range-Doppler 二维图，横轴为径向速度，纵轴为距离。
- `plots.json`：图片来源、FFT 文件哈希和绘图尺度说明。

三张图的 dB 都相对于各自峰值，使用 `10*log10(power/peak)`，不是绝对 dBm。
图中保留零多普勒回波，速度正负沿用 FFT 符号约定。

已有 `fft_results.npz` 时，可只补生成图片，无需重算 FFT：

```bash
cd rawData_capture
.venv/bin/python test.py --plot-only

# 指定数据对应的已有验证目录
.venv/bin/python test.py dataSet/adc_validation_20261119_02.npy --plot-only
```

`--no-plots` 可仅执行数值检查。
绘图依赖记录在 `requirements-plot.txt`，其离线安装包已保存在本项目 `wheelhouse/`；
`setup.sh` 会一并安装，采集本身不导入绘图库。
