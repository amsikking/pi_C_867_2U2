# pi_C_867_2U2
Python device adaptor: Physik Instrumente C-867.2U2 motion controller, for two-axis ultrasonic piezo stages.
## Details:
- The controller has a 'startup macro' set by the factory that runs on power up (default name 'AUTO'?).
- The macro is a sequence of 'GCS' commands that are executed on the controller to (hopefully) start the controller (and connected axes) in a known state. This macro can be viewed and modified (gulp!) either directly or in PIMikroMove for example (controller macros tab).
- One of the default commands in the startup macro is 'FRF = Fast Reference Move To Reference Switch' which effectively homes the stage and is required for any absolute move commands using 'MOV' (used in the device adaptor).
- Current conclusion:
  -  **the axes must 'home' on startup**
  -  **this can be annoying** (if for example a sample is on an XY stage).
