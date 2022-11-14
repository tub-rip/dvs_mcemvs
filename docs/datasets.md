### Datasets
- HKUST and edited RPG-ECCV 18 datasets [ESVO 2020](https://sites.google.com/view/esvo-project-page/home#h.tl1va3u667ae)
- [RPG-ECCV 2018](http://rpg.ifi.uzh.ch/ECCV18_stereo_davis.html)
- [MVSEC, RAL 2018](https://daniilidis-group.github.io/mvsec/)
- [DSEC](https://dsec.ifi.uzh.ch/). Additionally, camera poses for some sequences are provided [in this directory](../data/DSEC).
- [TUM-VIE](https://vision.in.tum.de/data/datasets/visual-inertial-event-dataset)

Our code expects ROSBags for input events and poses. To convert event data from HDF5 files (as in DSEC, TUM-VIE) to ROSBags quickly, you may use [our events_h52bag C++ utility](https://github.com/tub-rip/events_h52bag).

We suggest pre-processing the event data using the [DVS hot pixel filter](https://github.com/cedric-scheerlinck/dvs_tools/tree/master/dvs_hot_pixel_filter). It outputs bag files with a `.filtered` extension.
