/* Copyright (c) 2013-2026 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "ereader_disk.h"

#include <mgba/core/core.h>
#include <mgba/gba/interface.h>
#include <mgba/internal/gba/gba.h>
#include <mgba-util/vfs.h>

#define EREADER_DISK_MAX 16

struct EReaderStrip {
	char* path;
	void* data;
	size_t size;
};

static struct mCore* eCore = NULL;
static struct EReaderStrip eStrips[EREADER_DISK_MAX];
static unsigned eCount = 1;
static unsigned eIndex = 0;
static bool eEjected = false;

/*
 * Sizes GBACartEReaderScan decodes. It returns silently on anything else, so a
 * strip of the wrong length would be a card that does nothing and says nothing;
 * refusing the replace puts the failure where a frontend can report it.
 */
static bool _sizeIsScannable(size_t size) {
	switch (size) {
	case 1308:
	case 1344:
	case 1872:
	case 2076:
	case 2112:
	case 2912:
	case 3520:
	case 5456:
		return true;
	default:
		return false;
	}
}

static void _clearStrip(struct EReaderStrip* strip) {
	free(strip->path);
	free(strip->data);
	strip->path = NULL;
	strip->data = NULL;
	strip->size = 0;
}

void EReaderDiskReset(void) {
	size_t i;
	for (i = 0; i < EREADER_DISK_MAX; ++i) {
		_clearStrip(&eStrips[i]);
	}
	eCore = NULL;
	eCount = 1;
	eIndex = 0;
	eEjected = false;
}

static bool _erSetEject(bool ejected) {
	/* Closing on a loaded strip is the scan. GBACartEReaderQueueCard copies the
	 * buffer into its own 16-slot queue, which the game drains as it asks for
	 * cards, so this neither blocks nor needs the game to be ready. */
	if (!ejected && eEjected && eCore && eIndex < eCount) {
		struct EReaderStrip* strip = &eStrips[eIndex];
		if (strip->data && strip->size) {
			GBACartEReaderQueueCard((struct GBA*) eCore->board, strip->data, strip->size);
		}
	}
	eEjected = ejected;
	return true;
}

static bool _erGetEject(void) {
	return eEjected;
}

static unsigned _erGetImageIndex(void) {
	return eIndex;
}

static bool _erSetImageIndex(unsigned index) {
	if (index >= eCount) {
		return false;
	}
	eIndex = index;
	return true;
}

static unsigned _erGetNumImages(void) {
	return eCount;
}

static bool _erReplaceImage(unsigned index, const struct retro_game_info* info) {
	if (index >= eCount) {
		return false;
	}
	struct EReaderStrip* strip = &eStrips[index];
	if (!info || !info->path) {
		/* A null info is how a frontend removes an image. */
		_clearStrip(strip);
		return true;
	}

	struct VFile* vf = VFileOpen(info->path, O_RDONLY);
	if (!vf) {
		return false;
	}
	ssize_t size = vf->size(vf);
	if (size <= 0 || !_sizeIsScannable((size_t) size)) {
		vf->close(vf);
		return false;
	}
	void* data = malloc((size_t) size);
	if (!data) {
		vf->close(vf);
		return false;
	}
	if (vf->read(vf, data, (size_t) size) != size) {
		free(data);
		vf->close(vf);
		return false;
	}
	vf->close(vf);

	_clearStrip(strip);
	strip->data = data;
	strip->size = (size_t) size;
	strip->path = strdup(info->path);
	return true;
}

static bool _erAddImageIndex(void) {
	if (eCount >= EREADER_DISK_MAX) {
		return false;
	}
	++eCount;
	return true;
}

static bool _erGetImagePath(unsigned index, char* path, size_t len) {
	if (index >= eCount || !eStrips[index].path || !path || !len) {
		return false;
	}
	strncpy(path, eStrips[index].path, len - 1);
	path[len - 1] = '\0';
	return true;
}

static bool _erGetImageLabel(unsigned index, char* label, size_t len) {
	if (index >= eCount || !eStrips[index].path || !label || !len) {
		return false;
	}
	/* The file's own name: a card is identified by its title, and the set names
	 * each strip after the card it came off. */
	const char* base = strrchr(eStrips[index].path, '/');
	const char* alt = strrchr(eStrips[index].path, '\\');
	if (alt > base) {
		base = alt;
	}
	base = base ? base + 1 : eStrips[index].path;
	strncpy(label, base, len - 1);
	label[len - 1] = '\0';
	return true;
}

void EReaderDiskPublish(struct mCore* core, retro_environment_t environCallback) {
	static const struct retro_disk_control_ext_callback callbacks = {
		.set_eject_state = _erSetEject,
		.get_eject_state = _erGetEject,
		.get_image_index = _erGetImageIndex,
		.set_image_index = _erSetImageIndex,
		.get_num_images = _erGetNumImages,
		.replace_image_index = _erReplaceImage,
		.add_image_index = _erAddImageIndex,
		.set_initial_image = NULL,
		.get_image_path = _erGetImagePath,
		.get_image_label = _erGetImageLabel,
	};

	EReaderDiskReset();
	eCore = core;

	unsigned version = 0;
	if (environCallback(RETRO_ENVIRONMENT_GET_DISK_CONTROL_INTERFACE_VERSION, &version) && version >= 1) {
		environCallback(RETRO_ENVIRONMENT_SET_DISK_CONTROL_EXT_INTERFACE, (void*) &callbacks);
	} else {
		/* The v0 struct is the ext struct's leading members, so the same
		 * function pointers serve a frontend that only knows the old one. */
		environCallback(RETRO_ENVIRONMENT_SET_DISK_CONTROL_INTERFACE, (void*) &callbacks);
	}
}
