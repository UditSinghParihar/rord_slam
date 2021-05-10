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