# MC-EMVS: Multi-Camera Event-based Multi-View Stereo 
This is the code for the journal paper [**Multi-Event-Camera Depth Estimation and Outlier Rejection by Refocused Events Fusion**](https://doi.org/10.1002/aisy.202200221), also known as **MC-EMVS: Multi-Camera Event-based Multi-View Stereo**,
by [Suman Ghosh](https://www.linkedin.com/in/suman-ghosh-a8762576/) and [Guillermo Gallego](https://sites.google.com/view/guillermogallego), published at Advanced Intelligent Systems.
<h2 align="left">
  
[Paper](https://arxiv.org/pdf/2207.10494) | [Video](https://youtu.be/o7Bxg9XlHmg) | [Poster](/docs/2022_MCEVMS_poster.pdf)
</h2>

[![Multi-Event-Camera Depth Estimation and Outlier Rejection by Refocused Events Fusion](docs/mcemvs_thumbnail.jpg)](https://youtu.be/o7Bxg9XlHmg)

If you use this work in your research, please cite it as follows:

```bibtex
@article{Ghosh22aisy,
  author = {Ghosh, Suman and Gallego, Guillermo},  
  title = {Multi-Event-Camera Depth Estimation and Outlier Rejection by Refocused Events Fusion},
  journal = {Advanced Intelligent Systems},
  year = {2022},
  doi = {10.1002/aisy.202200221}
}
```

## Data Processing Pipeline


![pipeline](docs/block_all.png)

### Input
* Events from multiple cameras
* Pose of camera rig
* Camera calibration (intrinsic, extrinsic, hand-eye) parameters

### Output
* Depth map
* Confidence map
* Point cloud
* Intermediate ray density maps / Disparity Space Images (DSI)

## Code
* [Installation instructions](docs/installation.md)
* [Running the code with various configurations](docs/running.md)
* [Datasets used](docs/datasets.md)
* [Running Examples](docs/examples.md)
* [Evaluation scripts](docs/evaluation.md)


## License

The license is available [here](Software_License_Agreement_TUB_dvs_mcemvs.pdf).

Follow-up works
-------
* **[Event-based Stereo Depth Estimation: A Survey](https://arxiv.org/pdf/2409.17680)**
* **[ES-PTAM: Event-based Stereo Parallel Tracking and Mapping](https://github.com/tub-rip/ES-PTAM)**

Additional Resources on Event-based Vision
-------
* [Research page (TU Berlin RIP lab)](https://sites.google.com/view/guillermogallego/research/event-based-vision)
* [Course at TU Berlin](https://sites.google.com/view/guillermogallego/teaching/event-based-robot-vision)
* [Event-based Vision: A Survey](http://rpg.ifi.uzh.ch/docs/EventVisionSurvey.pdf)
* [List of Resources](https://github.com/uzh-rpg/event-based_vision_resources)
