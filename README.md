# Adaptive Cruise Control simulation for Carla 0.9.12
Author: Jakub Burczyk
## Requirements
* Carla version ```0.9.12``` available from https://github.com/carla-simulator/carla/releases/tag/0.9.12
* Python version ```3.7```
## Running ACC
* Run Carla 0.9.12
* Navigate to ```acc_scripts/``` and run ```main.py```

Windows command example:
```py -3.7 main.py -x maps/test_1.xodr --no-rendering --distance 10```

## Command line arguments:
| Arg  | Full arg | Desc | Val | Default |
|-|-|-|-|-|
| -x | --xodr-path | Loading OpenDRIVE map from given file | Relative path | maps/test_1.xodr |
| N/A | --no-rendering | Disable server 3D scene rendering | N/A | N/A |
| N/A | --res | Change 2D visualization window resolution | WIDTHxHEIGHT | 1280x720 |
| N/A | --save-csv | Save simulation data to csv files to disk| N/A | N/A |
| N/A | --save-plots | Save matplotlib plots of simulation data to disk | N/A | N/A |
| N/A | --function | Specify NPC vehicle speed function | "const" / "sine" / "square" | const |
| N/A | --const-vel | Constant component of NPC vehicle speed | ```float``` | 10.0 |
| N/A | --amplitude | Amplitude of NPC vehicle function (not applicable to "const") | ```float``` | 5.0 |
| N/A | --freq | Frequency of NPC vehicle function (not applicable to "const") | ```float``` | 0.05 |
| -d | --distance | Setpoint distance of ego to NPC vehicle given in meters | ```float``` | 15.0 |
| -vel | --velocity | Setpoint velocity for ego while not using ACC given in meters per second| ```float``` | 15.0 |

## Setting regulator params:
Setting regulator params is done via command line args:

| Arg  | Full arg | Desc | Val |
|-|-|-|-|
| -xy | --pid-x-y | Gain of given ```x``` regulator ```y``` component | ```float```|

Substitute the ```x``` character for regulator type: ```v``` to set velocity regulator params or ```d``` for distance regulator params.

Substitute the ```y``` character for regulator component: ```p``` for proportional gain, ```i``` for integral gain and ```d``` for derivative gain.

#### Example use:

* Setting velocity proportional gain to ```0.1``` : ```-vp 0.1``` or ```-pid-v-p 0.1```
* Setting distance integral gain to ```0.5``` : ```-di 0.5``` or ```-pid-d-i 0.5```

#### Default regulator params:
| Regulator| P | I | D |
| - | - | - | - |
| Velocity | 0.13 | 0.03 | 0.1 |
| Distance | 0.03 | 0.001 | 0.1 |

## Important Notes
Created PyGame window may be unresponsive for a few seconds after starting the scripts and display "Rendering Map" this is expected behavior.

If for some reason the script times out not connecting to the CARLA server please make sure it is running and re-run the script, this may be caused by loading map files.

Full map view mode with TAB key. Exit with ESC in visu window or CTRL+C in cmd.
