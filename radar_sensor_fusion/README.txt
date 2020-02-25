Implementation steps for the 2D CFAR process:
The 2D CFAR process is like application of a smoothing kernel (with a gap in the middle due to guard cells) across the matrix obtained from 2D FFT. For application of this, the configuration parameters, i.e. the training and guard cell width for both dimensions is set. In addition a little preprocessing is done to enlist all the indices w.r.t to the center of the matrix that need to be averaged. These are the cells that fall within in the training bracket. After this, the application is simply a loop in a buffered region of the matrix. At each iteration, the pre-calculated index offsets and index of the member in consideration are used to calculate the required average and the offset is applied.

Selection of Training, Guard cells and offset.
An appropriate combination is found by trial-and-error such that a clean target is obtained. Initial values are set taking into account the range of the 2D FFT in that dimension and the slope of the target peak edges.

Steps taken to suppress the non-thresholded cells at the edges.
All new matrices are initialized as a zero matrix of appropriate dimension and are modified only under the buffered loop. This leaves the edge cells untouched and maintains their value at 0.