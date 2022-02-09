# bluesky-rogue-scenariomaker
BlueSky scenario maker for rogue aircraft.

To generate scenarios
 1. modify the defaults in ```config.py```.
 2. run ```main.py```.

The module generates random paths with origins and destinations outside of open airspace. The cruising altitude of an aircraft changes three times during the path.

# Path examples

Two different paths can be generated.

Paths that respect constrained airspace are like this:

![image](https://user-images.githubusercontent.com/78442543/150139109-a6c464c9-81d7-48ad-86cf-618e53118544.png)

Paths that do not respect constrained airspace are like this:

![image](https://user-images.githubusercontent.com/78442543/150139523-0c85804e-d6ed-494d-bc74-33acebbb9763.png)

# Integration with concept scenarios
These [scenarios](https://github.com/Metropolis-2/bluesky-rogue-scenariomaker/tree/main/scenarios) can be added to any concept scenario by adding the ```PCALL``` command to it (see the BlueSky [documentation](https://github.com/TUDelft-CNS-ATM/bluesky/wiki/pcall) for more information).

Since concepts cannot control rogue aircraft, plugins contain exceptions for rogue aircraft. Rogue aircraft are able to navigate to their destinations without any input from the concepts.

There are two ways to identify a rogue aircraft:
1. The ```acid``` will always start with a capital "R".
2. The traffic array, ```bs.traf.roguetraffic.rogue_bool``` (see [here](https://github.com/Metropolis-2/bluesky/blob/1853587c6bcf626287c3b58cc972fe6eddb51af8/plugins/rogueaircraft.py#L55)) will be `True` for rogue aircraft.
