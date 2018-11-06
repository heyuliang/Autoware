Original repository can be found here: https://github.com/kujason/ip_basic

## Image Processing for Basic Depth Completion (IP-Basic)
IP Basic is a depth completion tool which converts a sparse depth map into a dense depth map. 
1. Run the script:
```bash
python depth_completion.py
```
This will run the algorithm on an example depth image for now and show you the result.

(Optional) Set options in `depth_completion.py`
- To run on the test set, comment the lines below `# Validation set` and uncomment the lines below `# Test set`
- `fill_type`:
  - `'fast'` - Version described in the paper
  - `'multiscale'` - Multi-scale dilations based on depth, with additional noise removal
- `extrapolate`:
  - `True`: Extends depths to the top of the frame and runs a 31x31 full kernel dilation
  - `False`: Skips the extension and large dilation
- `show_process`:
  - `True` - Shows the filling process. Only works with `fill_type == 'multiscale'`
  - `False` - Don't show it
  
