local_src := $(subdirectory)/src/src/opus.c
local_src += $(subdirectory)/src/src/opus_encoder.c
local_src += $(subdirectory)/src/src/opus_decoder.c
local_src += $(subdirectory)/src/src/repacketizer.c
local_src += $(subdirectory)/src/src/analysis.c
local_src += $(subdirectory)/src/src/mlp.c
local_src += $(subdirectory)/src/src/mlp_data.c
local_src += $(wildcard $(subdirectory)/src/src/*.s)
local_src += $(wildcard $(subdirectory)/src/celt/*.c)
local_src += $(wildcard $(subdirectory)/src/celt/*.s)
local_src += $(wildcard $(subdirectory)/src/silk/*.c)
local_src += $(wildcard $(subdirectory)/src/silk/fixed/*.c)
local_src += $(wildcard $(subdirectory)/src/silk/*.s)
local_src += $(wildcard $(subdirectory)/src/*.c)

includes_api += $(subdirectory)/src
includes_api += $(subdirectory)/src/celt
includes_api += $(subdirectory)/src/silk
includes_api += $(subdirectory)/src/silk/fixed
includes_api += $(subdirectory)/src/src
includes_api += $(subdirectory)/src/include

local_bin := $(BINDIR)/$(subdirectory)
bindirs   += $(local_bin)

$(eval $(call make-library, $(local_bin)/opus14.a, $(local_src)))