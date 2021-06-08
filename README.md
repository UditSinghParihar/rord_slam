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
		2. `python register.py --rgb1 ~/backup/d2-net/data_gazebo/data5/rgb/rgb000318.jpg  --rgb2 ~/backup/d2-net/data_gazebo/data5/rgb/rgb001439.jpg  --depth1 ~/backup/d2-net/data_gazebo/data5/depth/depth000318.npy  --depth2 ~/backup/d2-net/data_gazebo/data5/depth/depth001439.npy  --camera_file ../configs/camera_gazebo.txt  --H ../configs/topH.npy  --model_rord ../models/rord.pth --viz3d`  

### Optimizing pose graph  
1. `cd pose_graph`  
2. Generating odometry edges `noise.g2o` using gazebo's odometer output `poses.txt`.  
	1. `python genG2o.py data5/poses.txt`  
3. Adding loop closure edges `loop_pairs.txt` to generated odometry edges `noise.g2o` to output `noise_lc.g2o`. Also optimizing odometry and loop closure edges stored in `noise_lc.g2o` to output `opt.g2o`.  
	1. `python optimizePose.py data5/noise.g2o data5/loop_pairs.txt`  
4. Converting RoRD transformations to Right-handed and odom frame transformation:
	1. `python cordTrans.py ../demo/transLC.npy`   