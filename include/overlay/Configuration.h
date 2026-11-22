// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "Calibration.h"

void LoadProfile(CalibrationContext &ctx);
void SaveProfile(CalibrationContext &ctx);

// Export/import the CalibrationContext as a standalone JSON file.
// ExportProfileToFile writes the current profile verbatim; nothing else
// changes. ImportProfileFromFile parses the file into ctx and persists
// it to the registry via SaveProfile. Both return true on success.
bool ExportProfileToFile(const CalibrationContext &ctx, const char *path);
bool ImportProfileFromFile(CalibrationContext &ctx, const char *path);
