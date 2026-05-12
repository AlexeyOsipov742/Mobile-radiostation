# napi_audio_dma

Kernel-space SPI audio bridge for this project.

Creates one char device:
- `/dev/napi_audio_dma0`

## Build

```bash
cd kernel/napi_audio_dma
make
```

## Load

```bash
sudo insmod napi_audio_dma.ko bus_num=2 chip_select=0 spi_mode=0
```

## Unload

```bash
sudo rmmod napi_audio_dma
```

## Runtime contract

1. Open `/dev/napi_audio_dma0` (`O_RDWR`).
2. `ioctl(fd, NAPI_AUDIO_DMA_IOC_CONFIG, &cfg)`:
   - `cfg.api_version = NAPI_AUDIO_DMA_API_VERSION`
   - `cfg.mode = NAPI_AUDIO_DMA_MODE_ADC` for capture (read)
   - `cfg.mode = NAPI_AUDIO_DMA_MODE_DAC` for playback (write)
   - `cfg.sample_rate_hz = 8000` (or your rate)
   - `cfg.spi_speed_hz = sample_rate_hz * 16` (or `0` for auto)
3. `ioctl(fd, NAPI_AUDIO_DMA_IOC_START)`
4. Use `read`/`write` audio blocks.
5. `ioctl(fd, NAPI_AUDIO_DMA_IOC_STOP)` before close.

## Notes

- MCP3201 and MCP4822 require chip-select framing for every 16-bit sample.
- The module sends one SPI message made of many 2-byte transfers, with CS toggled between samples.
- A single long SPI transfer is not valid for these chips because CS stays asserted for the whole block.
- If the SPI bus/chip-select is already occupied by another device (e.g. `spidev`), binding may fail. In that case unbind conflicting consumer or adjust bus/cs.
