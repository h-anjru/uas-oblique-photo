# uas-oblique-photo
A simulation of oblique photogrammetry from the UAS platform for accuracy assessment and mission planning.

## Overview
I wrote these scripts as part of my dissertation research into oblique photogrammetry from drones over forests. Collecting oblique imagery in airborne photogrammetry has been practiced for a long time for ubran areas because it produces better 3D reconstructions of vertical features such as walls. But I had noticed a dearth of studies of oblique photogrammetry from UAS over forests, and I thought it warranted further study.

### Dynamic, close-range photogrammetry
Conventional aerial photogrammetry practices assume a much higher flying height above ground than those achieved using drones, yet most missions for UAS photogrammetry were being planned using those practices. My thought was that, instead of thinking of it as low-altitude aerial photogrammetry, we should think of drone photogrammetry as *dynamic, close-range photogrammetry.*

![Capturing nadir photos from low altitudes seemed to result in "false positives," or reconstruction of features that simply weren't there. Shown here is a 3D reconstrution of a cluster of trees (left: true color; right: color by height (blue:low > red:high).](figures/false-positives.png)

*Capturing nadir photos from low altitudes seemed to result in "false positives," or reconstruction of features that simply weren't there. Shown here is a 3D reconstrution of a cluster of trees (left: true color; right: color by height (blue:low > red:high).*

I considered the much-shorter average range from camera to target when using a drone versus a manned aircraft. The relief displacement is much mroe exxagerated. In other words, there is an increased target height to flying height ratio (for example, the height of a building in relation to the height of the camera above it is drastically different from a drone than a conventional manned aircraft).

![Uncertainty in reconstruction (Hartley & Zisserman)](figures/uncertainty-in-reconstruction_hartley-zisserman.png)

*Uncertainty in reconstruction (Hartley & Zisserman)*

The two main principles of close-range photogrammetry are

1. varying ranges of targets and 
2. strong geometry between camera stations.

Both of these can be acheived by capturing oblique images. To go further, both can be achieved using a single camera flown at a constant height above ground.

### Workflow
Applying these principles to drone photogrammetry should increase the accuracy of the resulting 3D reconstruction of the photos. My thought on the fastest way to test this contention was to run a few simulations of low-altitude, oblique photo capture in MATLAB.

1. Create a matrix of points in object space )roughly the height of the trees typically seen in my study areas)
2. Set an array of camera stations above these object space points at a given height and tilt
3. Back-project the object space points onto the camera stations--essentially creating virutal 2D images of the point array
4. Apply a bundle adjustment on these virtual images and the object space points and analyze the statistics on the adjustment

![Animation of the simulated environment created in MATLAB for testing oblique photo collection from low altitudes.](figures/oblique-flightlines.gif)

*Animation of the simulated environment created in MATLAB for testing oblique photo collection from low altitudes.*

### Supplemental information

You can read more about the simulations, the results, and thir application to real-world photo collects [here](/supplemental).

## missionSim.m
`missionSim()` creates a small simulated photo collection mission for testing bundle adjustment results with BUN2013.exe (not included in this repository).

### Call
`missionSim(endlap, sidelap, flying_height, tilt)`

### Input
`endlap`: endlap from photo to photo (0.0-1.0)

`sidelap`: sidelap from flight line to flight line (0.0-1.0)

`flying_height`: height above ground (meters)

`tilt`: angle of tilt of camera from nadir (deg)

### Output
The output is an ASCII file (.dat) which is formatted for input into a bundle adjustment software, BUN2013.exe, written by Bon Dewitt.

Reference: Wolf, Dewitt, Wilkinson. "Elements of Photogrammetry 4th ed."

## backProject.m
`backProject` back projects some array of object space coordinates into two images. This function assumes an ideal pinhole camera. Noise is added to the back-projected image coordinates.

### Call
`[x1, x2, noise] = backproject(X, cam1, cam2, sigma)`

### Input
`X`: `[n, n x 3]` matrix of `n` names, object space points of the format `[name, X, Y, Z]`

`cam`: row vector of external orientation parameters (EOPs) for camera `[XL, YL, ZL, omega, phi, kappa, f(pix)]`

`sigma`: stdev of image measurements (typically 0.5 pixel) used to crete Gaussian noise

### Output
`x`: `2 x n` matrix of image coordinates for cameras 1 & 2 `[x1, y1, x2, y2]`

Reference: Wolf, Dewitt, Wilkinson. "Elements of Photogrammetry 4th ed." Appendix D-4.