/* Copyright (c) 2013-2026 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef LIBRETRO_EREADER_DISK_H
#define LIBRETRO_EREADER_DISK_H

#include <mgba-util/common.h>

CXX_GUARD_START

#include "libretro.h"

struct mCore;

/*
 * Publishes the e-Reader's card slot to the frontend as libretro disk control.
 *
 * A dotcode strip is removable media: the frontend replaces the current image
 * with a strip and closes the tray, and the tray-close edge is what queues the
 * card for the game to scan. Nothing here is a disc; disk control is simply the
 * only interface libretro has for handing a running core a new medium.
 *
 * Call once after core->reset(), and only when the loaded cartridge has
 * HW_EREADER: a frontend told about disk control shows a disc menu for it.
 */
void EReaderDiskPublish(struct mCore* core, retro_environment_t environCallback);

/* Drop every strip and reset the slot. Safe to call without a publish. */
void EReaderDiskReset(void);

CXX_GUARD_END

#endif
