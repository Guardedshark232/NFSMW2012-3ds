# NFS Most Wanted 2012 - 3DS Build System

DEVKITPRO ?= /opt/devkitpro
DEVKITARM ?= $(DEVKITPRO)/devkitARM
CTRULIB   ?= $(DEVKITPRO)/libctru

TARGET = nfs_3ds
APP_TITLE = NFS Most Wanted 2012
APP_DESCRIPTION = Need for Speed: Most Wanted (2012) 3DS
APP_AUTHOR = Criterion / 3DS Port
APP_ICON = $(CTRULIB)/default_icon.png

# Compiler Flags
ARCH := -march=armv6k -mtune=mpcore -mfloat-abi=hard -mtp=soft
CFLAGS := -O2 $(ARCH) -g -Wall -ffunction-sections -fdata-sections -flto
CXXFLAGS := $(CFLAGS) -fno-rtti -fno-exceptions -std=gnu++14
ASFLAGS := -g $(ARCH)
LDFLAGS := -specs=3dsx.specs -g $(ARCH) -flto -Wl,--gc-sections -Wl,-Map,$(TARGET).map

# Include Directories
INCLUDES := -I./src -I./src/core -I./src/input -I./src/renderer -I./src/physics \
            -I$(DEVKITARM)/include -I$(CTRULIB)/include

# Libraries
LIBS := -L$(CTRULIB)/lib -lcitro2d -lcitro3d -lctru -lm

# Sources
SRCS_CORE := $(wildcard src/core/*.cpp) $(wildcard src/core/*.c)
SRCS_RENDER := $(wildcard src/renderer/*.cpp) $(wildcard src/renderer/*.c)
SRCS_INPUT := $(wildcard src/input/*.cpp) $(wildcard src/input/*.c)
SRCS_PHYSICS := $(wildcard src/physics/*.cpp) $(wildcard src/physics/*.c)

SRCS := $(SRCS_CORE) $(SRCS_RENDER) $(SRCS_INPUT) $(SRCS_PHYSICS)
OBJS := $(SRCS:.cpp=.o)
OBJS := $(OBJS:.c=.o)
OBJS += src/renderer/vshader_shbin.o

ROMFS_DIR := romfs

# Tools
MAKEROM := WINEPREFIX="$(CURDIR)/.wine" WINEDEBUG=-all wine "$(CURDIR)/NFSU2-3DS/tools/3ds-bin/makerom.exe"
BANNERTOOL := WINEPREFIX="$(CURDIR)/.wine" WINEDEBUG=-all wine "$(CURDIR)/NFSU2-3DS/tools/3ds-bin/bannertool.exe"

.PHONY: all cia clean minimap

all: $(TARGET).3dsx

cia: $(TARGET).cia

$(TARGET).smdh:
	$(DEVKITPRO)/tools/bin/smdhtool --create "$(APP_TITLE)" "$(APP_DESCRIPTION)" "$(APP_AUTHOR)" $(APP_ICON) $@

$(TARGET).3dsx: $(TARGET).elf $(TARGET).smdh romfs/assets/world_atlas.t3x
	$(DEVKITPRO)/tools/bin/3dsxtool $(TARGET).elf $(TARGET).3dsx --smdh=$(TARGET).smdh --romfs=$(ROMFS_DIR)

$(TARGET).cia: $(TARGET).elf meta/icon.icn meta/banner.bnr meta/app.rsf
	$(MAKEROM) -f cia -o $@ -DAPP_ENCRYPTED=false -target t \
		-elf $(TARGET).elf -icon meta/icon.icn -banner meta/banner.bnr \
		-rsf meta/app.rsf

meta/banner.bnr: meta/banner.png meta/silence.wav
	$(BANNERTOOL) makebanner -i meta/banner.png -a meta/silence.wav -o $@

meta/icon.icn: meta/icon.png
	$(BANNERTOOL) makesmdh -s "$(APP_TITLE)" -l "$(APP_DESCRIPTION)" -p "$(APP_AUTHOR)" -i meta/icon.png -o $@

$(TARGET).elf: $(OBJS)
	$(DEVKITARM)/bin/arm-none-eabi-g++ $(LDFLAGS) $(OBJS) -o $@ $(LIBS)

src/renderer/vshader.shbin: src/renderer/vshader.v.pica
	$(DEVKITPRO)/tools/bin/picasso -o $@ $<

src/renderer/vshader_shbin.s: src/renderer/vshader.shbin
	$(DEVKITPRO)/tools/bin/bin2s $< > $@

src/renderer/vshader_shbin.o: src/renderer/vshader_shbin.s
	$(DEVKITARM)/bin/arm-none-eabi-gcc -c $< -o $@

# World texture atlas: downscaled game albedos packed into one RGBA4444 page.
# The extractor bakes this atlas' UV rectangles into fairhaven_block.mwmesh.
tools/world_atlas.png: tools/world_texture_map.json tools/build_atlas.py tools/mwtex.py
	python3 tools/build_atlas.py

romfs/assets/world_atlas.t3x: tools/world_atlas.png
	$(DEVKITPRO)/tools/bin/tex3ds -f rgba4444 -z auto -o $@ $<

%.o: %.cpp
	$(DEVKITARM)/bin/arm-none-eabi-gcc $(CXXFLAGS) $(INCLUDES) -c $< -o $@

%.o: %.c
	$(DEVKITARM)/bin/arm-none-eabi-gcc $(CFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -f $(OBJS) src/renderer/vshader.shbin src/renderer/vshader_shbin.s \
		$(TARGET).3dsx $(TARGET).elf $(TARGET).map $(TARGET).smdh $(TARGET).cia \
		meta/banner.bnr meta/icon.icn

# Regenerate the top-down radar texture from the converted world mesh.
minimap:
	python3 tools/gen_minimap.py romfs/assets/fairhaven_block.mwmesh /tmp/minimap.png
	$(DEVKITPRO)/tools/bin/tex3ds -f rgba8 -z auto -o romfs/assets/minimap.t3x /tmp/minimap.png
