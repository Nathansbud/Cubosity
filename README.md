# Cubosity

A beautiful beautiful CSCI 2240 final project based on [Cubic Stylization](https://www.dgp.toronto.edu/projects/cubic-stylization/) (Liu & Jacobson, 2019), implemented by one Zack Amiton & [Stewart Morris](https://github.com/stew2003)

## Installation

## glew

TODO

## OSQP

Polyhedral generalization uses [OSQP](https://osqp.org/docs/) as its quadratic solver. We use the [osqp-eigen](https://github.com/robotology/osqp-eigen) wrapper for easy Eigen compatibility; both can be installed via conda by running

`conda install -c conda-forge osqp-eigen`

though more complete installation instructions can be found on the package websites.

## Setup

## Formats

Collapse sequences are stored in `.stamp` files, which have the following structure:

```bash
I $InitialFaceResolution $FinalFaceResolution
C $CollapsedEID R $Removed S $Shifted F $TopFID $BotFID W $TopVID $BotVID N $NeighborEID ... 
.
.
.
```
where `$Variables` are values, and uppercase letters are to help with parsing. `VID`, `EID`, and `FID` stand for vertex, edge, and face ID respectively. Further, `$Removed` and `$Shifted` are shorthands for the following structure:

- `$Removed` = `$RemovedEID $RemovedVID $RemovedVertex.X $RemovedVertex.Y $RemovedVertex.Z`
- `$Shifted` = `$ShiftedEID $ShiftedVertexVID $ShiftedVertex.X $ShiftedVertex.Y $ShiftedVertex.Z`

though while all `Removed` elements are actually removed, while only `ShiftedEID` is removed during collapse (`ShiftedVertex` is simply moved). Additionally, each `NeighborEID` corresponds with an edge which had its endpoint moved from `RemovedVertex` to `ShiftedVertex` during the collapse.
