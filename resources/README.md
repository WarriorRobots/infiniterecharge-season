# Resources readme

<!-- Author: Joshua Budd -->
<!-- Date: 2020/01/18 -->
<!-- Revised: 2020/02/10 -->

This is an information document to describe the contents of the resources
folder.

## Table of Contents

- [README.md](#README.md)
- [Guide](#Guide)
- [Wiring guide](#Wiring-guide)
- [Wiring layout](#Wiring-layout)
- [Button Binding](#Button-bindings)
- [Path Weaver tool](#Path-Weaver-tool)
- [Pipelines](#Pipelines)

## README.md

This document.

## Guide

A reference [guide](./guide.md) on how to do certain things pertaining to the robot code.

## Wiring guide

[Open here.](./wiring-guide.md)

A guide with information about wiring and electrical related items. (Composed of a .md and .html file and images folder.)

## Wiring layout

A csv file that contains the electronics plugged into the PDP and the information about them. This file is a csv so that changes can be seen in plain text in the git history.

Some notes about the file:
- Ports are labled as H (for 40 Amp breakers) and L (for 5-30 Amp breakers)
- IDs should start with the port number followed by a hyphen and (1 or) 2 letters describing the controller. eg. 01-RB stands for Right-Back Drive
- Values are left blank on purose instead of being filled with NA
- Excel may cover cells due to poor formatting as Excel struggles with non-xlsx files.

## Button Bindings

A csv file (./button-bidning-draft.csv) to keep track of button bindings.

## Path Weaver tool

[This](./movepathweaver.sh) is a tool to move the pathweaver output to be deployed on the robot.

## Pipelines

[Folder](./pipelines/) of pipelines for the Limelight. All files should be saved as `Title_Date_Time_change.json` / `TITLE_YYYYMMDD_HHMM_CHANGE.json`. (Old pielines should go into ./pipelines/old/)
