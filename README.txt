Name:
=====
 audio-ai-apollo-soc

Vendor File Patching (fit_main.c):
=================================
The Cordio BLE stack's FIT profile main file lives in the SDK tree at:
  third_party/cordio/ble-profiles/sources/apps/fit/fit_main.c

This file is outside the project's git boundary, so it cannot be tracked
directly. The patch at patches/fit_main.patch carries the project's
customizations: JSON service, Opus audio stream service, CCC table entries,
MTU negotiation, and the Opus timer pump.

Setup (after cloning):
----------------------
1. Check patch status:
     make vendor-patch-check

   Possible outputs:
     "Check OK: patch can be applied"  -- clean SDK, ready to patch
     "Patch already applied"           -- nothing to do
     "Patch check failed"              -- target file has unexpected changes

2. Apply the patch:
     make vendor-patch-apply

3. Build as usual:
     make -C gcc

Non-standard SDK location:
--------------------------
If this project is not inside the default AmbiqSuite directory layout,
point to the SDK root:
  AMBIQSUITE_ROOT=/path/to/AmbiqSuite_R4.3.0 make vendor-patch-apply

Reverting the patch:
--------------------
  make vendor-patch-revert

This restores fit_main.c to the original SDK version.

Updating the patch after editing fit_main.c:
--------------------------------------------
If you modify fit_main.c directly in the SDK tree, regenerate the patch
so others get your changes:

  diff -u /path/to/original/fit_main.c \
       third_party/cordio/ble-profiles/sources/apps/fit/fit_main.c \
    | sed 's|--- .*/fit_main.c|--- third_party/cordio/ble-profiles/sources/apps/fit/fit_main.c|;
           s|+++ .*/fit_main.c|+++ third_party/cordio/ble-profiles/sources/apps/fit/fit_main.c|' \
    > patches/fit_main.patch

Keep a copy of the unmodified SDK fit_main.c as your diff baseline.

HYER after patching, you need to manually update fit_main.c to improve opus audio download speed over BLE. Look at the "fitUpdateCfg change to fit_main.c.txt" file for more detail!

