#
#  Copyright (c) 2014 Arduino.  All right reserved.
#
#  This library is free software; you can redistribute it and/or
#  modify it under the terms of the GNU Lesser General Public
#  License as published by the Free Software Foundation; either
#  version 2.1 of the License, or (at your option) any later version.
#
#  This library is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#  See the GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public
#  License along with this library; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#
ARM_GCC_TOOLCHAIN=C:\Program Files (x86)\Arduino\hardware\tools\gcc-arm-none-eabi-4.8.3-2014q1\bin

# Compilation tools
AR = $(ARM_GCC_TOOLCHAIN)/arm-none-eabi-ar
CC = $(ARM_GCC_TOOLCHAIN)/arm-none-eabi-gcc
#CXX = $(ARM_GCC_TOOLCHAIN)/arm-none-eabi-g++
CXX = $(ARM_GCC_TOOLCHAIN)/arm-none-eabi-gcc
AS = $(ARM_GCC_TOOLCHAIN)/arm-none-eabi-as
GDB = $(ARM_GCC_TOOLCHAIN)/arm-none-eabi-gdb
SIZE = $(ARM_GCC_TOOLCHAIN)/arm-none-eabi-size
NM = $(ARM_GCC_TOOLCHAIN)/arm-none-eabi-nm
OBJCOPY = $(ARM_GCC_TOOLCHAIN)/arm-none-eabi-objcopy

RM=rm -Rf

SEP=\\

# ---------------------------------------------------------------------------------------
# C Flags

CFLAGS += -Wall -Wchar-subscripts -Wcomment -Wformat=2 -Wimplicit-int
CFLAGS += -Werror-implicit-function-declaration -Wmain -Wparentheses
CFLAGS += -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs -Wunused
CFLAGS += -Wuninitialized -Wunknown-pragmas -Wfloat-equal -Wundef
CFLAGS += -Wshadow -Wpointer-arith -Wbad-function-cast -Wwrite-strings
CFLAGS += -Wsign-compare -Waggregate-return -Wstrict-prototypes
CFLAGS += -Wmissing-prototypes -Wmissing-declarations
CFLAGS += -Wformat -Wmissing-format-attribute -Wno-deprecated-declarations
CFLAGS += -Wpacked -Wredundant-decls -Wnested-externs -Winline -Wlong-long
CFLAGS += -Wunreachable-code
CFLAGS += -Wcast-align

# -flto
CFLAGS += --param max-inline-insns-single=500 -mcpu=cortex-m0plus -mthumb -mlong-calls -ffunction-sections -nostdlib -std=c99
CFLAGS += $(OPTIMIZATION) $(INCLUDES) -D$(DEVICE) -D$(VARIANT)

# To reduce application size use only integer printf function.
CFLAGS += -Dprintf=iprintf
# -u _scanf_float -u _printf_float



# ---------------------------------------------------------------------------------------
# CPP Flags

CPPFLAGS += -Wall -Wchar-subscripts -Wcomment -Wformat=2
CPPFLAGS += -Wmain -Wparentheses -Wcast-align -Wunreachable-code
CPPFLAGS += -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs -Wunused
CPPFLAGS += -Wuninitialized -Wunknown-pragmas -Wfloat-equal -Wundef
CPPFLAGS += -Wshadow -Wpointer-arith -Wwrite-strings
CPPFLAGS += -Wsign-compare -Waggregate-return -Wmissing-declarations
CPPFLAGS += -Wformat -Wmissing-format-attribute -Wno-deprecated-declarations
CPPFLAGS += -Wpacked -Wredundant-decls -Winline -Wlong-long

#-fno-rtti -fno-exceptions -flto
CPPFLAGS += --param max-inline-insns-single=500 -mcpu=cortex-m0plus -mthumb -mlong-calls -ffunction-sections -fdata-sections -std=c++98
CPPFLAGS += $(OPTIMIZATION) $(INCLUDES) -D$(DEVICE)

# To reduce application size use only integer printf function.
CPPFLAGS += -Dprintf=iprintf



# ---------------------------------------------------------------------------------------
# ASM Flags

ASFLAGS = -mcpu=cortex-m0plus -mthumb -Wall -g $(OPTIMIZATION) $(INCLUDES)



# ---------------------------------------------------------------------------------------
# LD Flags

#--nostartfiles  -flto
LDFLAGS= -mcpu=cortex-m0plus -mthumb -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--entry=Reset_Handler -Wl,--unresolved-symbols=report-all -Wl,--warn-common -Wl,--warn-section-align -Wl,--warn-unresolved-symbols

