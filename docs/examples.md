## Running Examples

### RPG_ECCV18_edited

Download the [rpg_monitor_edited.bag](https://drive.google.com/file/d/1P8N3YfYnF5lgOgZGqkMU73otEnedztgy/view?usp=drive_web) file from the [ESVO 2020](https://sites.google.com/view/esvo-project-page/home#h.tl1va3u667ae), which contains stereo events and camera poses.
	
Set the correct path of the filtered bag file in the `--bag_filename` parameter in the file `mapper_emvs_stereo/cfg/rpg_eccv18/monitor_edited/alg1/rpg_monitor_edited_fixedts.conf`. Make sure that the topic names are correct. Finally, run mapper_emvs_stereo
	
	roscd mapper_emvs_stereo
	cd cfg/rpg_eccv18/monitor_edited/alg1/
	rosrun mapper_emvs_stereo run_emvs --flagfile=rpg_monitor_edited_fixedts.conf
	
The output files will be saved in the current directory. 
The raw depth points are stored in `014.000000depth_points_fused_2.txt`file in the format `[row column depth]`. 
The color-coded inverse depth map is saved as `014.000000inv_depth_colored_dilated_fused_2.png`. 
The suffix `_2` denotes the fusion function used (Harmonic mean -HM- in this case).

<table border="0" style="width:100%; border:none; border-collapse: collapse;">
  <tr style="border:none;">
    <td align="center" style="border:none;"><img src="https://user-images.githubusercontent.com/35840258/189180563-aaf4d421-af20-42d8-b27c-de87a7ae4d91.png" width="300"></td>
    <td align="center" style="border:none;"><img src="https://user-images.githubusercontent.com/35840258/189387315-9839b205-420e-4302-b215-1315472412a5.png" width="300"></td>
  </tr>
  <tr style="border:none;">
    <td align="center" style="border:none;">Depth map</td>
    <td align="center" style="border:none;">Confidence map</td>
  </tr>
</table>


### TUM-VIE

From the [TUM-VIE dataset](https://vision.in.tum.de/data/datasets/visual-inertial-event-dataset), download the following files:
* [mocap-desk2-events_left.h5](https://tumevent-vi.vision.in.tum.de/mocap-desk2/mocap-desk2-events_left.h5)
* [mocap-desk2-events_right.h5](https://tumevent-vi.vision.in.tum.de/mocap-desk2/mocap-desk2-events_right.h5) 
* [mocap-desk2-vi_gt_data.tar.gz](https://tumevent-vi.vision.in.tum.de/mocap-desk2/mocap-desk2-vi_gt_data.tar.gz) for camera poses.
* [camera-calibrationA.json](https://tumevent-vi.vision.in.tum.de/camera-calibrationA.json) for camera and hand-eye calibration.


Convert left and right events from h5 format to ROSBag. 
Clone and install [our h52bag converter](https://github.com/tub-rip/events_h52bag). Then, convert using:

	./events_h52bag mocap-desk2-events_left.h5 mocap-desk2-events_left /dvs/left/events 720 1280 700000000
	./events_h52bag mocap-desk2-events_right.h5 mocap-desk2-events_right /dvs/left/events 720 1280 700000000
	
This should generate 2 bag files for the events, namely `mocap-desk2-events_left_0.bag` and `mocap-desk2-events_right_0.bag`. This was tested with 32GB RAM. If you run out of memory, use a lower number for `events_per_bag` instead `700000000`. This will split the output into multiple ROSBag files.

Extract the contents of `mocap-desk2-vi_gt_data.tar.gz` into a folder `mocap-desk2-vi_gt_data`. Then, convert poses from `mocap_data.txt` to ROSBag using [this script](../mapper_emvs_stereo/scripts/mocap_txt2bag.py):
	
	python mapper_emvs_stereo/scripts/mocap_txt2bag.py --path_prefix mocap-desk2-vi_gt_data

This should generate `pose.bag` as output inside the `mocap-desk2-vi_gt_data` folder.

Set the correct path of the input events, poses and the calibration file by editing the configuration file `mapper_emvs_stereo/cfg/tumvie/desk2_full/tum-vie.conf`.

Finally, run mapper_emvs_stereo:

	roscd mapper_emvs_stereo
	cd cfg/rpg_eccv18/tumvie/desk2_full
	rosrun mapper_emvs_stereo run_emvs --flagfile=tum-vie.conf
	
This will process the whole desk2 sequence and generate a sequence of time-stamped output files (depth maps and confidence maps) in the current folder.

<table border="0" style="width:100%; border:none; border-collapse: collapse;">
  <tr style="border:none;">
    <td align="center" style="border:none;"><img src="https://user-images.githubusercontent.com/35840258/198369362-bde31d70-a449-420b-ae47-b785b3735225.gif" width="300"></td>
    <td align="center" style="border:none;"><img src="https://user-images.githubusercontent.com/35840258/198369369-bf25a99d-24dd-4d69-9501-d1f2dd8a00b5.gif" width="300"></td>
  </tr>
  <tr style="border:none;">
    <td align="center" style="border:none;">Depth map</td>
    <td align="center" style="border:none;">Confidence map</td>
  </tr>
</table>

### DSEC

From the [DSEC dataset](https://dsec.ifi.uzh.ch/dsec-datasets/download/), download the following files:
* [interlaken_00_b_events_left.zip](https://download.ifi.uzh.ch/rpg/DSEC/test/interlaken_00_b/interlaken_00_b_events_left.zip)
* [interlaken_00_b_events_right.zip](https://download.ifi.uzh.ch/rpg/DSEC/test/interlaken_00_b/interlaken_00_b_events_right.zip) 
* [interlaken_00_b_calibration.zip](https://download.ifi.uzh.ch/rpg/DSEC/test/interlaken_00_b/interlaken_00_b_calibration.zip) for camera and hand-eye calibration.

Camera poses obtained using LiDAR-IMU odometry are available [here](data/DSEC/interlaken_00-odometry/pose.bag) in ROSBag format. Thanks to [Mathias Gehrig](https://magehrig.github.io/) for the data.

Extract the zip files. Convert left and right events from h5 format to ROSBag. 
Clone and install [our h52bag converter](https://github.com/tub-rip/events_h52bag). Then, convert using:

	./events_h52bag interlaken_00_b_events_left/events.h5 interlaken_00_b_events_left /dvs/left/events 480 640
	./events_h52bag interlaken_00_b_events_right/events.h5 interlaken_00_b_events_right /dvs/right/events 480 640
	
Since each h5 file is big (>500M events), this will generate multiple ROSBag files containing events from both cameras. In this example, we'll use the files `interlaken_00_b_events_left_1.bag` and `interlaken_00_b_events_right_1.bag`, which comprises a subset of the whole `interlaken_00_b` sequence.

Set the correct path of the input events, poses and the unzipped calibration files by editing the configuration file `mapper_emvs_stereo/cfg/DSEC/interlaken_00_b_2/dsec.conf`.

Finally, run mapper_emvs_stereo:

	roscd mapper_emvs_stereo
	cd cfg/DSEC/interlaken_00_b_2
	rosrun mapper_emvs_stereo run_emvs --flagfile=dsec.conf
	
This will process the subsequence `interlaken_00_b_1` and generate a sequence of time-stamped output files (depth maps and confidence maps) in the current folder.

<table border="0" style="width:100%; border:none; border-collapse: collapse;">
  <tr style="border:none;">
    <td align="center" style="border:none;"><img src="https://user-images.githubusercontent.com/35840258/201677608-9a7b7d0a-9eef-465f-8ffb-dc0205ca111e.gif" width="300"></td>
    <td align="center" style="border:none;"><img src="https://user-images.githubusercontent.com/35840258/201679065-03a26bc1-8d15-47f5-800d-d57cbc7ec29b.gif" width="300"></td>
  </tr>
  <tr style="border:none;">
    <td align="center" style="border:none;">Depth map</td>
    <td align="center" style="border:none;">Confidence map</td>
  </tr>
</table>
