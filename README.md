# Introduction

This is a repository forked from 'betaflight/betaflight version 3.3.x', we modified the functionalities of the original flight control.

## Alitutude

For altitude, we only used the measurement of 'TF MINI' for indoor navigation. For altitude hold, we replaceded the series control 'ALT_P->VEL_PID' of betaflight with our own 'ALT_PI+VEL' (the newest version of betaflightit also removed the 'ALT_HOLD').

## Telemetry

We connected the flight controller with computation unit 'Jevois', we modified 'telemetry.c' for the communication (MAVLINK meaasges) between high and low controllers. There maybe a better way to realize this, such as sharing serial port ('TELEMETRY' and 'RX').

