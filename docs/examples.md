## Running Examples

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
