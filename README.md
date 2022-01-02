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

## Important Notes
Created PyGame window may be unresponsive for a few seconds after starting the scripts and display "Rendering Map" this is expected behavior.

If for some reason the script times out not connecting to the CARLA server please make sure it is running and re-run the script, this may be caused by loading map files.

Full map view mode with TAB key. Exit with ESC in visu window or CTRL+C in cmd.
