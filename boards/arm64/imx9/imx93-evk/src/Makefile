############################################################################
# boards/arm64/imx9/imx93-evk/src/Makefile
#
# SPDX-License-Identifier: Apache-2.0
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

CSRCS = imx9_boardinit.c imx9_bringup.c

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += imx9_appinit.c
endif

ifeq ($(CONFIG_PWM),y)
CSRCS += imx9_pwm.c
endif

ifeq ($(CONFIG_IMX9_LPI2C),y)
CSRCS += imx9_i2c.c
endif

ifeq ($(CONFIG_IMX9_LPSPI),y)
CSRCS += imx9_spi.c
endif

ifeq ($(CONFIG_IMX9_USDHC),y)
CSRCS += imx9_usdhc.c
endif

include $(TOPDIR)/boards/Board.mk

ifeq ($(CONFIG_IMX9_BOOTLOADER),y)
.PHONY: distclean
distclean::
	$(call DELFILE, ${TOPDIR}${DELIM}imx9-sdimage.img)
	$(call DELFILE, ${TOPDIR}${DELIM}imx9-norimage.img)
endif
