#include <stdint.h>

#include "am_util.h"
#include "fake_opus_audio.h"

static uint8_t g_fakeOpusData[FAKE_OPUS_TOTAL_BYTES];
static uint32_t g_fakeOpusChecksum = 0;
static uint8_t g_fakeOpusReady = 0;

static uint32_t next_lfsr(uint32_t x)
{
  uint32_t lsb = x & 1U;
  x >>= 1;
  if (lsb != 0U)
  {
    x ^= 0xD0000001U;
  }
  return x;
}

void FakeOpusAudioInit(void)
{
  uint32_t lfsr = 0x1ACEB00CU;
  uint32_t i;
  uint32_t frame;
  uint32_t frameBase;

  if (g_fakeOpusReady != 0U)
  {
    return;
  }

  g_fakeOpusChecksum = 0;

  for (frame = 0; frame < FAKE_OPUS_FRAME_COUNT; frame++)
  {
    frameBase = frame * FAKE_OPUS_FRAME_BYTES;

    /* Fake frame marker + low frame index to make stream inspection easier. */
    g_fakeOpusData[frameBase + 0U] = 0xF8U;
    g_fakeOpusData[frameBase + 1U] = (uint8_t)(frame & 0xFFU);
    g_fakeOpusChecksum += g_fakeOpusData[frameBase + 0U];
    g_fakeOpusChecksum += g_fakeOpusData[frameBase + 1U];

    for (i = 2U; i < FAKE_OPUS_FRAME_BYTES; i++)
    {
      uint32_t sampleIdx = frameBase + i;
      uint32_t phase = (sampleIdx * 440U * 256U) / 16000U;
      uint8_t tone = ((phase & 0x80U) != 0U) ? 0xC0U : 0x40U;
      uint8_t noise;
      uint8_t out;

      lfsr = next_lfsr(lfsr);
      noise = (uint8_t)(lfsr & 0x3FU);
      out = (uint8_t)(tone ^ noise);

      g_fakeOpusData[frameBase + i] = out;
      g_fakeOpusChecksum += out;
    }
  }

  g_fakeOpusReady = 1U;

  am_util_debug_printf("Fake Opus generated at boot\r\n");
  am_util_debug_printf("  duration: %u sec\r\n", (unsigned)FAKE_OPUS_DURATION_SEC);
  am_util_debug_printf("  bitrate : %u bps\r\n", (unsigned)FAKE_OPUS_BITRATE_BPS);
  am_util_debug_printf("  size    : %u bytes\r\n", (unsigned)FAKE_OPUS_TOTAL_BYTES);
  am_util_debug_printf("  frames  : %u x %u bytes\r\n",
                       (unsigned)FAKE_OPUS_FRAME_COUNT,
                       (unsigned)FAKE_OPUS_FRAME_BYTES);
  am_util_debug_printf("  checksum: 0x%08X\r\n", (unsigned)g_fakeOpusChecksum);
}

const uint8_t *FakeOpusAudioGetData(uint32_t *pLen)
{
  if (pLen != NULL)
  {
    *pLen = FAKE_OPUS_TOTAL_BYTES;
  }

  return g_fakeOpusData;
}
