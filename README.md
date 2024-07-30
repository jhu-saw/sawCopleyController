
# sawCopleyController

This SAW package contains a component (mtsCopleyController) that interfaces to a Copley Controller via a serial (RS232) interface. It has been tested with a Copley Xenus Plus single-axis controller, specifically XEC-230-15. The software has been written to support multi-axis controllers, but that feature has not been tested.

Most of the source code is in the [core](./core) subdirectory to facilitate building with ROS1 or ROS2.

The component is designed to be generic, and is configured using a JSON file, which can specify a CCX file for controller configuration. Sample JSON and CCX files are in the [core/share](./core/share) sub-directory.

Copley controllers are configured using parameters that are typically referred to by their hexadecimal addresses. Many parameters exist in both RAM and Flash. On power-up, or after reset, the controller copies the values from Flash to RAM. This component assumes that either the correct parameter values have previously been stored in Flash, or the correct values are available in a CCX file, which the component will parse during startup (if `load_ccx` is true) and set the parameter values in RAM. The CCX file can be generated manually or by vendor tools. Documentation about the CCX file format can be found online.

There are two approaches to instantiate this component:

1. Create the component and specify the `port_name` and `baud_rate` in the configuration file (JSON). As a safety check, specify `axis_label` in the configuration file, which will be compared to the value obtained from the drive (parameter 0x92); if they do not agree, most functionality will be disabled.

```
std::string axis_name = "AxisName";
mtsCopleyController *copleyServer = new mtsCopleyController(axis_name);
std::string config_file = axis_name+".json";
copleyServer->Configure(config_file);
```

2. Specify the `port_name` and `baud_rate` in the constructor, read the axis label (parameter 0x92) from the drive, then configure using the desired JSON file. In the following code, the axis label is used to rename the component and determine the correct config file, but the programmer can instead specify a mapping between axis_label and component name. In this approach, it is not necessary to specify `port_name`, `baud_rate` or `axis_label` in the JSON file.

```
mtsCopleyController *copleyServer = new mtsCopleyController("TEMP", "COM1", 115200);
std::string axis_label = copleyServer->GetAxisLabel();
copleyServer->SetName(axis_label);
std::string config_file = axis_label+".json";
copleyServer->Configure(config_file);
```

The disadvantage of the first approach is that the serial port is specified in the JSON file; on some operating systems, the serial ports may be reassigned when the system is power-cycled. The disadvantage of the second approach (besides its greater complexity) is that the axis label (parameter 0x92) must be programmed in the Flash memory of the Copley controller.

The JSON file contains the following fields, where DU indicates Display Units (e.g., millimeters, degrees), rather than SI units (e.g., meters, radians):

| Keyword      | Default   | Description                                     |
|:-------------|:----------|:------------------------------------------------|
| file_version | 0         | Version of JSON file format                     |
| name         |           | Descriptive name                                |
| port_name    | ""        | Serial port name (e.g., COM1)                   |
| baud_rate    | 9600      | Serial port baud rate                           |
| is_plus      | true      | Whether a Plus controller                       |
| ccx_file     | ""        | CCX file to configure controller                |
| load_ccx     | true      | Whether to load CCX file on startup             |
| axes         |           | Array of axis configuration data (see below)    |
|  - type      |           | - "PRISMATIC" or "REVOLUTE"                     |
|  - position_bits_to_SI |  | - conversion scale and offset (*)              |
|  -- scale    | 1         | -- scale factor (bits to SI)                    |
|  -- offset   | 0         | -- offset (DU)                                  |
|  - home_pos  | 0         | - home position, in DU (**)                     |
|  - position_limits |     | - upper and lower joint position limits         |
|  -- lower    | -MAX      | -- lower position limit (DU)                    |
|  -- upper    | +MAX      | -- upper position limit (DU)                    |
|  - axis_label | ""       | - axis label on drive (parameter 0x92)          |

(*) The conversion (position_bits_to_SI) is applied as follows:  value_SI = value_bits/scale + offset_SI, where offset_SI is computed by converting offset from Display Units (mm or deg) to SI (m or rad). Thus, value_bits = (value_SI - offset_SI)*scale. Note that the offset is only used for positions, not for velocities, accelerations or decelerations.

(**) The home position is specified in the JSON file, rather than the CCX file, and is used to set the home offset parameter (0xc6) on the drive; any setting of 0xc6 in the CCX file is ignored. Note that the software negates the home_pos value because the Copley controller treats it as an offset TO the zero position, rather than as an offset FROM the zero position.
