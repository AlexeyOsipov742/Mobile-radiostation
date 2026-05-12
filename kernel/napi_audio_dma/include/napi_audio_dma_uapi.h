#pragma once

#include <linux/ioctl.h>
#include <linux/types.h>

typedef __u8  napi_dma_u8;
typedef __u16 napi_dma_u16;
typedef __u32 napi_dma_u32;

#define NAPI_AUDIO_DMA_IOC_MAGIC 0xAD
#define NAPI_AUDIO_DMA_API_VERSION 1u

enum {
    NAPI_AUDIO_DMA_MODE_ADC = 1,
    NAPI_AUDIO_DMA_MODE_DAC = 2,
};

enum {
    NAPI_AUDIO_DMA_F_ADC_PARSE_MCP3201 = 1u << 0,
    NAPI_AUDIO_DMA_F_DAC_CH_B          = 1u << 1,
};

struct napi_audio_dma_config {
    napi_dma_u32 api_version;
    napi_dma_u32 mode;
    napi_dma_u32 sample_rate_hz;
    napi_dma_u32 spi_speed_hz;
    napi_dma_u32 flags;
    napi_dma_u32 reserved[3];
};

#define NAPI_AUDIO_DMA_IOC_CONFIG _IOW(NAPI_AUDIO_DMA_IOC_MAGIC, 1, struct napi_audio_dma_config)
#define NAPI_AUDIO_DMA_IOC_START  _IO(NAPI_AUDIO_DMA_IOC_MAGIC, 2)
#define NAPI_AUDIO_DMA_IOC_STOP   _IO(NAPI_AUDIO_DMA_IOC_MAGIC, 3)
