# pi_C_867_2U2
Python device adaptor: Physik Instrumente C-867.2U2 motion controller, for two-axis ultrasonic piezo stages.

![social_preview](https://github.com/amsikking/pi_C_867_2U2/blob/main/social_preview.png)

## Details:
- The controller has a 'startup macro' set by the factory that runs on power up (default name 'AUTO'?).
- The macro is a sequence of 'GCS' commands that are executed on the controller to (hopefully) start the controller (and connected axes) in a known state. This macro can be viewed and modified (gulp!) either directly or in PIMikroMove for example (controller macros tab).
- One of the default commands in the startup macro is 'FRF = Fast Reference Move To Reference Switch' which effectively homes the stage and is required for any absolute move commands using 'MOV' (used in the device adaptor).
- Current conclusion:
  -  **the axes must 'home' on startup**
  -  **this can be annoying** (if for example a sample is on an XY stage).
  
**Note:** the PI provided **joystick** connects directly to the controller and therefore _bypasses_ the PC. This means the **position attributes (self.x, self.y) are outdated** when the joystick is used. To minimize latency and maximize user control the device adaptor will **not** update the attributes (i.e. there is no loop or subprocess allocated to this). The burden is on the user of the adapter to update the attributes at the correct time, by calling either 'self.get_position_mm()' or 'self.move_mm'. Calling 'self.get_position_mm()' takes about ~3ms and can be launched in a thread for minimum latency. However, if the controller is _busy_ during a call to 'self.get_position_mm()' it will _not block_, but instead respond with the position limit (x_min, y_min, x_max, y_max) indicating it's current direction of travel. A Python enabled joystick could avoid this issue.
