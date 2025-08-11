############################################################################
# boards/arm/ra8/fpb-ra8e1/kernel/Makefile
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

include $(TOPDIR)/Make.defs

# The entry point name (if none is provided in the .config file)

CONFIG_INIT_ENTRYPOINT ?= user_start
ENTRYPT = $(patsubst "%",%,$(CONFIG_INIT_ENTRYPOINT))

# Get the paths to the libraries and the links script path in format that
# is appropriate for the host OS

USER_LIBPATHS = $(addprefix -L,$(call CONVERT_PATH,$(addprefix $(TOPDIR)$(DELIM),$(dir $(USERLIBS)))))
USER_LDLIBS = $(patsubst lib%,-l%,$(basename $(notdir $(USERLIBS))))
USER_LIBGCC = "${shell "$(CC)" $(ARCHCPUFLAGS) -print-libgcc-file-name}"

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  BOARDMAKE = $(if $(wildcard .\board.mk),.\board.mk,$(TOPDIR)$(DELIM)board$(DELIM)board.mk)
else
  BOARDMAKE = $(if $(wildcard ./board.mk),./board.mk,$(TOPDIR)/board/board.mk)
endif

# Get the board-specific linker script

ifeq ($(CONFIG_BUILD_PROTECTED),y)
LDSCRIPT = kernel-space.ld
else ifeq ($(CONFIG_BUILD_KERNEL),y)
LDSCRIPT = kernel-space.ld
else
LDSCRIPT = flash.ld
endif

# Create the list of available programs and add the path to the generated
# program

AVAILABLE_APPS := $(shell ls -1 $(BINDIR))
AVAILABLE_APPS := $(filter-out .*, $(AVAILABLE_APPS))
AVAILABLE_APPS := $(foreach APP, $(AVAILABLE_APPS), $(BINDIR)$(DELIM)$(APP))

# Determine which linker script to use
LD_SCRIPT = $(call CONVERT_PATH,$(TOPDIR)$(DELIM)boards$(DELIM)$(CONFIG_ARCH)$(DELIM)$(CONFIG_ARCH_CHIP)$(DELIM)$(CONFIG_ARCH_BOARD)$(DELIM)scripts$(DELIM)$(LDSCRIPT))

# Create the list of libraries to be linked in

LDLIBS = $(patsubst %.a,%,$(patsubst lib%,-l%,$(USERLIBS)))

# The object files that must be linked to create the application

LDSTARTGROUP ?= --start-group
LDENDGROUP ?= --end-group

RA8E1_LIBS = $(USER_LIBPATHS) $(LDSTARTGROUP) $(USER_LDLIBS) $(LDENDGROUP) $(USER_LIBGCC)
RA8E1_EXTRA = $(EXTRA_LIBS)

# Targets:

all: $(TOPDIR)$(DELIM)nuttx.elf $(TOPDIR)$(DELIM)User.map
.PHONY: depend clean distclean

# Create the nuttx.elf file containing all of the user-mode code

nuttx.elf: $(AVAILABLE_APPS)
	$(Q) echo "LD: nuttx.elf"
	$(Q) $(LD) --gc-sections -T$(LD_SCRIPT) -Map=$(TOPDIR)$(DELIM)User.map -o $@ $(RA8E1_LIBS) $(RA8E1_EXTRA)
	$(Q) $(NM) -u $@ && exit 1 || true
	$(Q) $(OBJDUMP) --syms $@ > $(TOPDIR)$(DELIM)User.sym
	$(Q) $(call CONVERT_PATH,$(TOPDIR)$(DELIM)tools$(DELIM)mkexport.sh) $@ $(AVAILABLE_APPS)

$(TOPDIR)$(DELIM)nuttx.elf: nuttx.elf
	$(Q) echo "CP: nuttx.elf"
	$(Q) $(CP) nuttx.elf $(TOPDIR)$(DELIM)nuttx.elf

$(TOPDIR)$(DELIM)User.map: nuttx.elf

# Dependencies

depend:

clean:
	$(call DELFILE, nuttx.elf)
	$(call DELFILE, User.map)
	$(call DELFILE, User.sym)
	$(call DELFILE, $(TOPDIR)$(DELIM)nuttx.elf)
	$(call DELFILE, $(TOPDIR)$(DELIM)User.map)

distclean: clean

-include board.mk
