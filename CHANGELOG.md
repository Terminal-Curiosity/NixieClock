# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Comprehensive README with project overview, features, hardware details, and getting started guide
- Project documentation structure with `docs/` folder for design docs and images
- ESP32-based Nixie tube clock firmware
- Auto-brightness adjustment via LDR sensor
- Dual PCF8574 I2C I/O expander support
- NeoPixel RGB LED status indicator
- Cathode protection through periodic tube refresh (2kHz)
- Three PCB designs: ControlDaughterBoard, NixieTubePanel, Monoboard (archived)
- Support for 4-digit nixie tube display (hours:minutes)
- Accurate timekeeping with high-resolution ESP32 timer
- Modular firmware architecture with separate components for clock, config, nixie control, and I/O

### Changed

- **Project Structure Reorganization**
  - Moved `Design Docs/` → `docs/design/`
  - Moved `Images/` → `docs/images/`
  - Renamed `PCB/` → `_PCB/` (archived for reference, not required for firmware builds)
- Updated `.gitignore` with cleaner organization and corrected paths
  - Removed old paths for deleted folders
  - Added Python cache patterns
  - Added IDE patterns (.idea, editor backups)
  - Grouped entries by category

### Removed

- `Datasheets/` folder (cleaned up workspace)
- Redundant entries in `.gitignore`
