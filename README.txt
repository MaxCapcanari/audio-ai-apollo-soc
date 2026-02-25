Name:
=====
 audio-ai-apollo-soc

Vendor File Patching (fit_main.c):
=================================
This repository depends on AmbiqSuite's external vendor file:
`third_party/cordio/ble-profiles/sources/apps/fit/fit_main.c`

Because that file is outside this git repo, custom changes are tracked here as:
`patches/fit_main.patch`

How to use after cloning:
-------------------------
1. From this repo root, run:
   `make vendor-patch-check`
2. Apply patch:
   `make vendor-patch-apply`

If your repo is not inside the default AmbiqSuite layout, set:
`AMBIQSUITE_ROOT=/path/to/AmbiqSuite_R4.3.0`

Example:
`AMBIQSUITE_ROOT=/opt/AmbiqSuite_R4.3.0 make vendor-patch-apply`

Revert patch if needed:
`make vendor-patch-revert`
