Radar_sense2GoL

===============================================================================
Description of project
===============================================================================
Motion detection FW including Micrium uC Probe GUI for Sense2GoL V1.2 board.

Features:
    - Motion is indicated via the three onboard LEDs:
        - Red LED: Target approaching RADAR
        - Orange LED: Target moving away from RADAR
        - Blue LED: No motion
    - Motion detection can be configured via Threshold and min/max velocity 
      settings.
    - I/Q raw data is streamed via UART interface 

===============================================================================
Change History                               
===============================================================================

2017-03-14:
    -  Initial version 1.0.0
    
2018-01-15:
    -  Version 1.1.0
        -  Added MATLAB support including doppler algorithm processing project
        -  UART streaming enabled by default
        -  UART device changed to IRQ mode
        -  Default sample size changed to 128 samples
        -  Fixed misspelling in GUI (MMWSW-133)
        -  updated documentation

===============================================================================
Known Issues
===============================================================================

None