# RoRD SLAM  

### Pretrained Models and dataset  

1. Download models from [Google Drive](https://drive.google.com/file/d/1-5aLHyZ_qlHFNfRnDpXUh5egtf_XtoiA/view?usp=sharing) (73.9 MB) in the base directory.    
2. Download dataset from [Google Drive](https://drive.google.com/file/d/1BkhcHBKwcjNHgbLZ1XKurpcP7v4hFD_b/view?usp=sharing) (97.8 MB)  

### Generating top view  
1. Selects four points in the image in the Region of interest, whose top view is required:  
	1. `python getRealOneGazebo.py ~/backup/d2-net/data_gazebo/data5/rgb/rgb000318.jpg ~/backup/d2-net/data_gazebo/data5/depth/depth000318.npy`  

### Caculating transformation  
1. Inferring on gazebo dataset in orthographic view:    
		1. `cd demo`  
		2. `python register.py --rgb1 <img1.jpg>  --rgb2 <img2.jpg>  --depth1 <depth1.npy>  --depth2 <depth2.npy>  --camera_file ../configs/camera_gazebo.txt  --H ../configs/topH.npy  --model_rord ../models/rord.pth --viz3d --save_trans`  
		3. If homography is different for the two images, then use `--H` and `--H2` flags:    
			1. `python register.py --H <first_homography.npy> --H2 <second_homography.npy>`    

2. Converting RoRD transformations (in camera frame and in left handed system) to loop closure transformations (in odom frame and in right handed system):  
	1. Getting static transform from ros, `Tbase_camera` or `Camera wrt Base link`
		1. `rosrun tf tf_echo base_link camera_link`    
	2. `python cordTrans.py --static_trans ../configs/camWrtBase.txt --rord_trans ../demo/transLC.npy`  
	3. Derivation of how transformations in [`cordTrans.py`](pose_graph/cordTrans.py) are dervied can be found [here](https://drive.google.com/file/d/1UfLmfj4JtnokyQDI0k9mx3KbO0Xsvegk/view?usp=sharing).  

### Optimizing pose graph  
1. `cd pose_graph`  
2. Generating odometry edges `noise.g2o` using gazebo's odometer output `poses.txt`.  
	1. `python genG2o.py data5/poses.txt`  
3. Adding loop closure edges `loop_pairs.txt` to generated odometry edges `noise.g2o` to output `noise_lc.g2o`. Also optimizing odometry and loop closure edges stored in `noise_lc.g2o` to output `opt.g2o`.  
	1. `python optimizePose.py data5/noise.g2o data5/loop_pairs.txt`  