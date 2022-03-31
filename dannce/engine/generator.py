"""Generator module for dannce training.
"""
from contextlib import suppress
import os
import numpy as np
from tensorflow import keras
from dannce.engine import processing as processing
from dannce.engine import ops as ops
from dannce.engine.video import LoadVideoFrame
import imageio
import warnings
import time
import scipy.ndimage.interpolation
import tensorflow as tf

# from tensorflow_graphics.geometry.transformation.axis_angle import rotate
from multiprocessing.dummy import Pool as ThreadPool
from typing import List, Dict, Tuple, Text

MISSING_KEYPOINTS_MSG = (
    "If mirror augmentation is used, the right_keypoints indices and left_keypoints "
    + "indices must be specified as well. "
    + "For the skeleton, ['RHand, 'LHand', 'RFoot', 'LFoot'], "
    + "set right_keypoints: [0, 2] and left_keypoints: [1, 3] in the config file"
)

TF_GPU_MEMORY_FRACTION = 0.9


class DataGenerator(keras.utils.Sequence):
    """Generate data for Keras.

    Attributes:
        batch_size (int): Batch size to generate
        camnames (List): List of camera names.
        clusterIDs (List): List of sampleIDs
        crop_height (Tuple): (first, last) pixels in image height
        crop_width (tuple): (first, last) pixels in image width
        currvideo (Dict): Contains open video objects
        currvideo_name (Dict): Contains open video object names
        dim_in (Tuple): Input dimension
        dim_out (Tuple): Output dimension
        extension (Text): Video extension
        indexes (np.ndarray): sample indices used for batch generation
        labels (Dict): Label dictionary
        list_IDs (List): List of sampleIDs
        mono (bool): If True, use grayscale image.
        n_channels_in (int): Number of input channels
        n_channels_out (int): Number of output channels
        out_scale (int): Scale of the output gaussians.
        samples_per_cluster (int): Samples per cluster
        shuffle (bool): If True, shuffle the samples.
        vidreaders (Dict): Dict containing video readers.
        predict_flag (bool): If True, use imageio for reading videos, rather than OpenCV
    """

    def __init__(
        self,
        list_IDs: List,
        labels: Dict,
        clusterIDs: List,
        batch_size: int = 32,
        dim_in: Tuple = (32, 32, 32),
        n_channels_in: int = 1,
        n_channels_out: int = 1,
        out_scale: float = 5,
        shuffle: bool = True,
        camnames: List = [],
        crop_width: Tuple = (0, 1024),
        crop_height: Tuple = (20, 1300),
        samples_per_cluster: int = 0,
        vidreaders: Dict = None,
        chunks: int = 3500,
        mono: bool = False,
        mirror: bool = False,
        predict_flag: bool = False,
    ):
        """Initialize Generator.
        """
        self.dim_in = dim_in
        self.dim_out = dim_in
        self.batch_size = batch_size
        self.labels = labels
        self.vidreaders = vidreaders
        self.list_IDs = list_IDs
        self.n_channels_in = n_channels_in
        self.n_channels_out = n_channels_out
        self.shuffle = shuffle
        # sigma for the ground truth joint probability map Gaussians
        self.out_scale = out_scale
        self.camnames = camnames
        self.crop_width = crop_width
        self.crop_height = crop_height
        self.clusterIDs = clusterIDs
        self.samples_per_cluster = samples_per_cluster
        self._N_VIDEO_FRAMES = chunks
        self.mono = mono
        self.mirror = mirror
        self.predict_flag = predict_flag
        self.on_epoch_end()

        if self.vidreaders is not None:
            self.extension = (
                "." + list(vidreaders[camnames[0][0]].keys())[0].rsplit(".")[-1]
            )

        assert len(self.list_IDs) == len(self.clusterIDs)

        self.load_frame = LoadVideoFrame(
            self._N_VIDEO_FRAMES, self.vidreaders, self.camnames, self.predict_flag
        )

    def __len__(self) -> int:
        """Denote the number of batches per epoch.

        Returns:
            int: Batches per epoch
        """
        return int(np.floor(len(self.list_IDs) / self.batch_size))

    def on_epoch_end(self):
        """Update indexes after each epoch."""
        self.indexes = np.arange(len(self.list_IDs))

        if self.shuffle:
            np.random.shuffle(self.indexes)


class DataGenerator_3Dconv(DataGenerator):
    """Update generator class to handle multiple experiments.

    Attributes:
        camera_params (Dict): Camera parameters dictionary.
        channel_combo (Text): Method for shuffling camera input order
        com3d (Dict): Dictionary of com3d data.
        COM_aug (bool): If True, augment the COM.
        crop_im (bool): If True, crop images.
        depth (bool): If True, appends voxel depth to sampled image features [DEPRECATED]
        dim_out_3d (Tuple): Dimensions of the 3D volume, in voxels
        distort (bool): If true, apply camera undistortion.
        expval (bool): If True, process an expected value network (AVG)
        gpu_id (Text): Identity of GPU to use.
        immode (Text): Toggles using 'video' or 'tif' files as image input [DEPRECATED]
        interp (Text): Interpolation method.
        labels_3d (Dict): Contains ground-truth 3D label coordinates.
        mode (Text): Toggles output label format to match MAX vs. AVG network requirements.
        multicam (bool): If True, formats data to work with multiple cameras as input.
        norm_im (bool): If True, normalize images.
        nvox (int): Number of voxels per box side
        rotation (bool): If True, use simple rotation augmentation.
        tifdirs (List): Directories of .tifs
        var_reg (bool): If True, adds a variance regularization term to the loss function.
        vmax (int): Maximum box dim (relative to the COM)
        vmin (int): Minimum box dim (relative to the COM)
        vsize (float): Side length of one voxel
        predict_flag (bool): If True, use imageio for reading videos, rather than OpenCV
    """

    def __init__(
        self,
        list_IDs: List,
        labels: Dict,
        labels_3d: Dict,
        camera_params: Dict,
        clusterIDs: List,
        com3d: Dict,
        tifdirs: List,
        batch_size: int = 32,
        dim_in: Tuple = (32, 32, 32),
        n_channels_in: int = 1,
        n_channels_out: int = 1,
        out_scale: int = 5,
        shuffle: bool = True,
        camnames: List = [],
        crop_width: Tuple = (0, 1024),
        crop_height: Tuple = (20, 1300),
        vmin: int = -100,
        vmax: int = 100,
        nvox: int = 32,
        gpu_id: Text = "0",
        interp: Text = "linear",
        depth: bool = False,
        channel_combo=None,
        mode: Text = "3dprob",
        samples_per_cluster: int = 0,
        immode: Text = "tif",
        rotation: bool = False,
        vidreaders: Dict = None,
        distort: bool = True,
        expval: bool = False,
        multicam: bool = True,
        var_reg: bool = False,
        COM_aug: bool = None,
        crop_im: bool = True,
        norm_im: bool = True,
        chunks: int = 3500,
        mono: bool = False,
        mirror: bool = False,
        predict_flag: bool = False,
    ):
        """Initialize data generator.

        Args:
            list_IDs (List): List of sample Ids
            labels (Dict): Dictionary of labels
            labels_3d (Dict): Dictionary of 3d labels.
            camera_params (Dict): Camera parameters dictionary.
            clusterIDs (List): List of sample Ids
            com3d (Dict): Dictionary of com3d data.
            tifdirs (List): Directories of .tifs
            batch_size (int, optional): Batch size to generate
            dim_in (Tuple, optional): Input dimension
            n_channels_in (int, optional): Number of input channels
            n_channels_out (int, optional): Number of output channels
            out_scale (int, optional): Scale of the output gaussians.
            shuffle (bool, optional): If True, shuffle the samples.
            camnames (List, optional): List of camera names.
            crop_width (Tuple, optional): (first, last) pixels in image width
            crop_height (Tuple, optional): (first, last) pixels in image height
            vmin (int, optional): Minimum box dim (relative to the COM)
            vmax (int, optional): Maximum box dim (relative to the COM)
            nvox (int, optional): Number of voxels per box side
            gpu_id (Text, optional): Identity of GPU to use.
            interp (Text, optional): Interpolation method.
            depth (bool): If True, appends voxel depth to sampled image features [DEPRECATED]
            channel_combo (Text): Method for shuffling camera input order
            mode (Text): Toggles output label format to match MAX vs. AVG network requirements.
            samples_per_cluster (int, optional): Samples per cluster
            immode (Text): Toggles using 'video' or 'tif' files as image input [DEPRECATED]
            rotation (bool, optional): If True, use simple rotation augmentation.
            vidreaders (Dict, optional): Dict containing video readers.
            distort (bool, optional): If true, apply camera undistortion.
            expval (bool, optional): If True, process an expected value network (AVG)
            multicam (bool): If True, formats data to work with multiple cameras as input.
            var_reg (bool): If True, adds a variance regularization term to the loss function.
            COM_aug (bool, optional): If True, augment the COM.
            crop_im (bool, optional): If True, crop images.
            norm_im (bool, optional): If True, normalize images.
            chunks (int, optional): Size of chunks when using chunked mp4.
            mono (bool, optional): If True, use grayscale image.
            predict_flag (bool, optional): If True, use imageio for reading videos, rather than OpenCV
        """
        DataGenerator.__init__(
            self,
            list_IDs,
            labels,
            clusterIDs,
            batch_size,
            dim_in,
            n_channels_in,
            n_channels_out,
            out_scale,
            shuffle,
            camnames,
            crop_width,
            crop_height,
            samples_per_cluster,
            vidreaders,
            chunks,
            mono,
            mirror,
            predict_flag,
        )
        self.vmin = vmin
        self.vmax = vmax
        self.nvox = nvox
        self.vsize = (vmax - vmin) / nvox
        self.dim_out_3d = (nvox, nvox, nvox)
        self.labels_3d = labels_3d
        self.camera_params = camera_params
        self.interp = interp
        self.depth = depth
        self.channel_combo = channel_combo
        print(self.channel_combo)
        self.gpu_id = gpu_id
        self.mode = mode
        self.immode = immode
        self.tifdirs = tifdirs
        self.com3d = com3d
        self.rotation = rotation
        self.distort = distort
        self.expval = expval
        self.multicam = multicam
        self.var_reg = var_reg
        self.COM_aug = COM_aug
        self.crop_im = crop_im
        # If saving npy as uint8 rather than training directly, dont normalize
        self.norm_im = norm_im

        # importing torch here allows other modes to run without pytorch installed
        self.torch = __import__("torch")
        self.device = self.torch.device("cuda:" + self.gpu_id)

        self.threadpool = ThreadPool(len(self.camnames[0]))

        ts = time.time()
        # Limit GPU memory usage by Tensorflow to leave memory for PyTorch
        config = tf.compat.v1.ConfigProto()
        config.gpu_options.per_process_gpu_memory_fraction = TF_GPU_MEMORY_FRACTION
        config.gpu_options.allow_growth = True
        self.session = tf.compat.v1.Session(config=config, graph=tf.Graph())
        print("Executing eagerly: ", tf.executing_eagerly(), flush=True)
        for i, ID in enumerate(list_IDs):
            experimentID = int(ID.split("_")[0])
            for camname in self.camnames[experimentID]:
                # M only needs to be computed once for each camera
                K = self.camera_params[experimentID][camname]["K"]
                R = self.camera_params[experimentID][camname]["R"]
                t = self.camera_params[experimentID][camname]["t"]
                M = self.torch.as_tensor(
                    ops.camera_matrix(K, R, t), dtype=self.torch.float32
                )
                self.camera_params[experimentID][camname]["M"] = M

        print("Init took {} sec.".format(time.time() - ts))

    def __getitem__(self, index: int):
        """Generate one batch of data.

        Args:
            index (int): Frame index

        Returns:
            Tuple[np.ndarray, np.ndarray]: One batch of data X
                (np.ndarray): Input volume y
                (np.ndarray): Target
        """
        # Generate indexes of the batch
        indexes = self.indexes[index * self.batch_size : (index + 1) * self.batch_size]

        # Find list of IDs
        list_IDs_temp = [self.list_IDs[k] for k in indexes]

        # Generate data
        X, y = self.__data_generation(list_IDs_temp)

        return X, y

    def project_grid(self, X_grid, camname, ID, experimentID):
        """Projects 3D voxel centers and sample images as projected 2D pixel coordinates

        Args:
            X_grid (np.ndarray): 3-D array containing center coordinates of each voxel.
            camname (Text): camera name
            ID (Text): string denoting a sample ID
            experimentID (int): identifier for a video recording session.

        Returns:
            np.ndarray: projected voxel centers, now in 2D pixels
        """
        ts = time.time()
        # Need this copy so that this_y does not change
        this_y = self.torch.as_tensor(
            self.labels[ID]["data"][camname],
            dtype=self.torch.float32,
            device=self.device,
        ).round()

        if self.torch.all(self.torch.isnan(this_y)):
            #com_precrop = self.torch.zeros_like(this_y[:, 0]) * self.torch.nan
            com_precrop = self.torch.zeros_like(this_y[:, 0])*float("nan")
        else:
            # For projecting points, we should not use this offset
            com_precrop = self.torch.mean(this_y, axis=1)

        this_y[0, :] = this_y[0, :] - self.crop_width[0]
        this_y[1, :] = this_y[1, :] - self.crop_height[0]
        com = self.torch.mean(this_y, axis=1)

        thisim = self.load_frame.load_vid_frame(
            self.labels[ID]["frames"][camname],
            camname,
            extension=self.extension,
        )[
            self.crop_height[0] : self.crop_height[1],
            self.crop_width[0] : self.crop_width[1],
        ]
        return self.pj_grid_post(
            X_grid, camname, ID, experimentID, com, com_precrop, thisim
        )

    def pj_grid_mirror(self, X_grid, camname, ID, experimentID, thisim):
        this_y = self.torch.as_tensor(
            self.labels[ID]["data"][camname],
            dtype=self.torch.float32,
            device=self.device,
        ).round()

        if self.torch.all(self.torch.isnan(this_y)):
            com_precrop = self.torch.zeros_like(this_y[:, 0]) * float("nan")
        else:
            # For projecting points, we should not use this offset
            com_precrop = self.torch.mean(this_y, axis=1)

        this_y[0, :] = this_y[0, :] - self.crop_width[0]
        this_y[1, :] = this_y[1, :] - self.crop_height[0]
        com = self.torch.mean(this_y, axis=1)

        if not self.mirror:
            raise Exception(
                "Trying to project onto mirrored images without mirror being set properly"
            )

        if self.camera_params[experimentID][camname]["m"] == 1:
            passim = thisim[-1::-1].copy()
        elif self.camera_params[experimentID][camname]["m"] == 0:
            passim = thisim.copy()
        else:
            raise Exception("Invalid mirror parameter, m, must be 0 or 1")

        return self.pj_grid_post(
            X_grid, camname, ID, experimentID, com, com_precrop, passim
        )

    def pj_grid_post(self, X_grid, camname, ID, experimentID, com, com_precrop, thisim):
        # separate the porjection and sampling into its own function so that
        # when mirror == True, this can be called directly
        if self.crop_im:
            if self.torch.all(self.torch.isnan(com)):
                thisim = self.torch.zeros(
                    (self.dim_in[1], self.dim_in[0], self.n_channels_in),
                    dtype=self.torch.uint8,
                    device=self.device,
                )
            else:
                thisim = processing.cropcom(thisim, com, size=self.dim_in[0])
        # print('Frame loading took {} sec.'.format(time.time() - ts))

        ts = time.time()
        proj_grid = ops.project_to2d_torch(
            X_grid, self.camera_params[experimentID][camname]["M"], self.device
        )
        # print('Project2d took {} sec.'.format(time.time() - ts))

        ts = time.time()
        if self.distort:
            proj_grid = ops.distortPoints_torch(
                proj_grid[:, :2],
                self.camera_params[experimentID][camname]["K"],
                np.squeeze(self.camera_params[experimentID][camname]["RDistort"]),
                np.squeeze(self.camera_params[experimentID][camname]["TDistort"]),
                self.device,
            )
            proj_grid = proj_grid.transpose(0, 1)
            # print('Distort took {} sec.'.format(time.time() - ts))

        ts = time.time()
        if self.crop_im:
            proj_grid = proj_grid[:, :2] - com_precrop + self.dim_in[0] // 2
            # Now all coordinates should map properly to the image cropped around the COM
        else:
            # Then the only thing we need to correct for is crops at the borders
            proj_grid = proj_grid[:, :2]
            proj_grid[:, 0] = proj_grid[:, 0] - self.crop_width[0]
            proj_grid[:, 1] = proj_grid[:, 1] - self.crop_height[0]

        rgb = ops.sample_grid_torch(thisim, proj_grid, self.device, method=self.interp)
        # print('Sample grid {} sec.'.format(time.time() - ts))

        if (
            ~self.torch.any(self.torch.isnan(com_precrop))
            or (self.channel_combo == "avg")
            or not self.crop_im
        ):
            X = rgb.permute(0, 2, 3, 4, 1)

        return X

    # TODO(nesting): There is pretty deep locigal nesting in this function,
    # might be useful to break apart
    def __data_generation(self, list_IDs_temp):
        """Generate data containing batch_size samples.
        X : (n_samples, *dim, n_channels)

        Args:
            list_IDs_temp (List): List of experiment Ids

        Returns:
            Tuple: Batch_size training samples
                X: Input volumes
                y_3d: Targets
                rotangle: Rotation angle
        Raises:
            Exception: Invalid generator mode specified.
        """
        # Initialization
        first_exp = int(self.list_IDs[0].split("_")[0])

        X = self.torch.zeros(
            (
                self.batch_size * len(self.camnames[first_exp]),
                *self.dim_out_3d,
                self.n_channels_in + self.depth,
            ),
            dtype=self.torch.uint8,
            device=self.device,
        )

        if self.mode == "3dprob":
            y_3d = self.torch.zeros(
                (self.batch_size, self.n_channels_out, *self.dim_out_3d),
                dtype=self.torch.float32,
                device=self.device,
            )
        elif self.mode == "coordinates":
            y_3d = self.torch.zeros(
                (self.batch_size, 3, self.n_channels_out),
                dtype=self.torch.float32,
                device=self.device,
            )
        else:
            raise Exception("not a valid generator mode")

        sz = self.dim_out_3d[0] * self.dim_out_3d[1] * self.dim_out_3d[2]
        X_grid = self.torch.zeros(
            (self.batch_size, sz, 3),
            dtype=self.torch.float32,
            device=self.device,
        )

        # Generate data
        for i, ID in enumerate(list_IDs_temp):
            sampleID = int(ID.split("_")[1])
            experimentID = int(ID.split("_")[0])

            # For 3D ground truth
            this_y_3d = self.torch.as_tensor(
                self.labels_3d[ID],
                dtype=self.torch.float32,
                device=self.device,
            )
            this_COM_3d = self.torch.as_tensor(
                self.com3d[ID], dtype=self.torch.float32, device=self.device
            )

            # Create and project the grid here,

            xgrid = self.torch.arange(
                self.vmin + this_COM_3d[0] + self.vsize / 2,
                this_COM_3d[0] + self.vmax,
                self.vsize,
                dtype=self.torch.float32,
                device=self.device,
            )
            ygrid = self.torch.arange(
                self.vmin + this_COM_3d[1] + self.vsize / 2,
                this_COM_3d[1] + self.vmax,
                self.vsize,
                dtype=self.torch.float32,
                device=self.device,
            )
            zgrid = self.torch.arange(
                self.vmin + this_COM_3d[2] + self.vsize / 2,
                this_COM_3d[2] + self.vmax,
                self.vsize,
                dtype=self.torch.float32,
                device=self.device,
            )
            (x_coord_3d, y_coord_3d, z_coord_3d) = self.torch.meshgrid(
                xgrid, ygrid, zgrid
            )
            
            if self.mode == "3dprob":
                for j in range(self.n_channels_out):
                    y_3d[i, j] = np.exp(
                        -(
                            (y_coord_3d - this_y_3d[1, j]) ** 2
                            + (x_coord_3d - this_y_3d[0, j]) ** 2
                            + (z_coord_3d - this_y_3d[2, j]) ** 2
                        )
                        / (2 * self.out_scale ** 2)
                    )
                    # When the voxel grid is coarse, we will likely miss
                    # the peak of the probability distribution, as it
                    # will lie somewhere in the middle of a large voxel.
                    # So here we renormalize to [~, 1]

            if self.mode == "coordinates":
                if this_y_3d.shape == y_3d[i].shape:
                    y_3d[i] = this_y_3d
                else:
                    msg = "Note: ignoring dimension mismatch in 3D labels"
                    warnings.warn(msg)

            X_grid[i] = self.torch.stack(
                (
                    x_coord_3d.transpose(0, 1).flatten(),
                    y_coord_3d.transpose(0, 1).flatten(),
                    z_coord_3d.transpose(0, 1).flatten(),
                ),
                axis=1,
            )

            # Compute projected images in parallel using multithreading
            ts = time.time()
            num_cams = len(self.camnames[experimentID])
            arglist = []
            if self.mirror:
                # Here we only load the video once, and then parallelize the projection
                # and sampling after mirror flipping. For setups that collect views
                # in a single imgae with the use of mirrors
                loadim = self.load_frame.load_vid_frame(
                    self.labels[ID]["frames"][self.camnames[experimentID][0]],
                    self.camnames[experimentID][0],
                    extension=self.extension,
                )[
                    self.crop_height[0] : self.crop_height[1],
                    self.crop_width[0] : self.crop_width[1],
                ]
                for c in range(num_cams):
                    arglist.append(
                        [
                            X_grid[i],
                            self.camnames[experimentID][c],
                            ID,
                            experimentID,
                            loadim,
                        ]
                    )
                result = self.threadpool.starmap(self.pj_grid_mirror, arglist)
            else:
                for c in range(num_cams):
                    arglist.append(
                        [X_grid[i], self.camnames[experimentID][c], ID, experimentID]
                    )
                result = self.threadpool.starmap(self.project_grid, arglist)

            for c in range(num_cams):
                ic = c + i * len(self.camnames[experimentID])
                X[ic, :, :, :, :] = result[c]
            # print('MP took {} sec.'.format(time.time()-ts))

        if self.multicam:
            X = X.reshape(
                (
                    self.batch_size,
                    len(self.camnames[first_exp]),
                    X.shape[1],
                    X.shape[2],
                    X.shape[3],
                    X.shape[4],
                )
            )
            X = X.permute((0, 2, 3, 4, 5, 1))

            if self.channel_combo == "avg":
                X = self.torch.mean(X, axis=-1)

            # Randomly reorder the cameras fed into the first layer
            elif self.channel_combo == "random":
                X = X[:, :, :, :, :, self.torch.randperm(X.shape[-1])]
                X = X.transpose(4, 5).reshape(*X.shape[:4], -1)
            else:
                X = X.transpose(4, 5).reshape(*X.shape[:4], -1)
        else:
            # Then leave the batch_size and num_cams combined
            y_3d = y_3d.repeat(num_cams, 1, 1, 1, 1)

        # 3dprob is required for *training* MAX networks
        if self.mode == "3dprob":
            y_3d = y_3d.permute([0, 2, 3, 4, 1])

        if self.mono and self.n_channels_in == 3:
            # Convert from RGB to mono using the skimage formula. Drop the duplicated frames.
            # Reshape so RGB can be processed easily.
            X = X.reshape(*X.shape[:4], len(self.camnames[first_exp]), -1)
            X = X[..., 0] * 0.2125 + X[..., 1] * 0.7154 + X[..., 2] * 0.0721

        # Convert pytorch tensors back to numpy array
        ts = time.time()
        if self.torch.is_tensor(X):
            X = X.float().cpu().numpy()
        if self.torch.is_tensor(y_3d):
            y_3d = y_3d.cpu().numpy()
        # print('Numpy took {} sec'.format(time.time() - ts))

        if self.expval:
            if self.torch.is_tensor(X_grid):
                X_grid = X_grid.cpu().numpy()
            if self.var_reg:
                return (
                    [processing.preprocess_3d(X), X_grid],
                    [y_3d, self.torch.zeros((self.batch_size, 1))],
                )

            if self.norm_im:
                # y_3d is in coordinates here.
                return [processing.preprocess_3d(X), X_grid], y_3d
            else:
                return [X, X_grid], y_3d
        else:
            if self.norm_im:
                return processing.preprocess_3d(X), y_3d
            else:
                return X, y_3d

def random_continuous_rotation(X, y_3d, max_delta=5):
    """Rotates X and y_3d a random amount around z-axis.

    Args:
        X (np.ndarray): input image volume
        y_3d (np.ndarray): 3d target (for MAX network) or voxel center grid (for AVG network)
        max_delta (int, optional): maximum range for rotation angle.

    Returns:
        np.ndarray: rotated image volumes
        np.ndarray: rotated grid coordimates
    """
    rotangle = np.random.rand() * (2 * max_delta) - max_delta
    X = tf.reshape(X, [X.shape[0], X.shape[1], X.shape[2], -1]).numpy()
    y_3d = tf.reshape(y_3d, [y_3d.shape[0], y_3d.shape[1], y_3d.shape[2], -1]).numpy()
    for i in range(X.shape[0]):
        X[i] = tf.keras.preprocessing.image.apply_affine_transform(
            X[i],
            theta=rotangle,
            row_axis=0,
            col_axis=1,
            channel_axis=2,
            fill_mode="nearest",
            cval=0.0,
            order=1,
        )
        y_3d[i] = tf.keras.preprocessing.image.apply_affine_transform(
            y_3d[i],
            theta=rotangle,
            row_axis=0,
            col_axis=1,
            channel_axis=2,
            fill_mode="nearest",
            cval=0.0,
            order=1,
        )

    X = tf.reshape(X, [X.shape[0], X.shape[1], X.shape[2], X.shape[2], -1]).numpy()
    y_3d = tf.reshape(
        y_3d,
        [y_3d.shape[0], y_3d.shape[1], y_3d.shape[2], y_3d.shape[2], -1],
    ).numpy()

    return X, y_3d


# TODO(inherit): Several methods are repeated, consider inheriting from parent
class DataGenerator_3Dconv_frommem(keras.utils.Sequence):
    """Generate 3d conv data from memory.

    Attributes:
        augment_brightness (bool): If True, applies brightness augmentation
        augment_continuous_rotation (bool): If True, applies rotation augmentation in increments smaller than 90 degrees
        augment_hue (bool): If True, applies hue augmentation
        batch_size (int): Batch size
        bright_val (float): Brightness augmentation range (-bright_val, bright_val), as fraction of raw image brightness
        chan_num (int): Number of input channels
        data (np.ndarray): Image volumes
        expval (bool): If True, crafts input for an AVG network
        hue_val (float): Hue augmentation range (-hue_val, hue_val), as fraction of raw image hue range
        indexes (np.ndarray): Sample indices used for batch generation
        labels (Dict): Label dictionary
        list_IDs (List): List of sampleIDs
        nvox (int): Number of voxels in each grid dimension
        random (bool): If True, shuffles camera order for each batch
        rotation (bool): If True, applies rotation augmentation in 90 degree increments
        rotation_val (float): Range of angles used for continuous rotation augmentation
        shuffle (bool): If True, shuffle the samples before each epoch
        var_reg (bool): If True, returns input used for variance regularization
        xgrid (np.ndarray): For the AVG network, this contains the 3D grid coordinates
        n_rand_views (int): Number of reviews to sample randomly from the full set
        replace (bool): If True, samples n_rand_views with replacement
        aux_labels (np.ndarray): If not None, contains the 3D MAX training targets for AVG+MAX training.
        temporal_chunk_list (np.ndarray, optional): If not None, contains chunked sampleIDs -- useful when loading in temporally contiguous samples
    """

    def __init__(
        self,
        list_IDs,
        data,
        labels,
        batch_size,
        rotation=True,
        random=True,
        chan_num=3,
        shuffle=True,
        expval=False,
        xgrid=None,
        var_reg=False,
        nvox=64,
        augment_brightness=True,
        augment_hue=True,
        augment_continuous_rotation=True,
        mirror_augmentation=False,
        right_keypoints=None,
        left_keypoints=None,
        bright_val=0.05,
        hue_val=0.05,
        rotation_val=5,
        replace=True,
        n_rand_views=None,
        heatmap_reg=False,
        heatmap_reg_coeff=0.01,
        aux_labels=None,
        temporal_chunk_list=None,
        separation_loss=False,
        symmetry_loss=False,
        gaussian_reg=False
    ):
        """Initialize data generator.
        """
        self.list_IDs = list_IDs
        self.data = data
        self.labels = labels
        self.rotation = rotation
        self.batch_size = batch_size
        self.random = random
        self.chan_num = chan_num
        self.shuffle = shuffle
        self.expval = expval
        self.augment_hue = augment_hue
        self.augment_continuous_rotation = augment_continuous_rotation
        self.augment_brightness = augment_brightness
        self.mirror_augmentation = mirror_augmentation
        self.right_keypoints = right_keypoints
        self.left_keypoints = left_keypoints
        if self.mirror_augmentation and (
            self.right_keypoints is None or self.left_keypoints is None
        ):
            raise Exception(MISSING_KEYPOINTS_MSG)
        self.var_reg = var_reg
        self.xgrid = xgrid
        self.nvox = nvox
        self.bright_val = bright_val
        self.hue_val = hue_val
        self.rotation_val = rotation_val
        self.n_rand_views = n_rand_views
        self.replace = replace
        self.heatmap_reg = heatmap_reg
        self.heatmap_reg_coeff = heatmap_reg_coeff
        self.aux_labels = aux_labels
        self.temporal_chunk_list = temporal_chunk_list
        self.temporal_chunk_size = 1
        self.temporal_batch_size = batch_size
        self.separation_loss=separation_loss
        self.symmetry_loss = symmetry_loss
        self.gaussian_reg = gaussian_reg
        self._update_temporal_batch_size()
        self.on_epoch_end()

    def __len__(self):
        """Denote the number of batches per epoch.

        Returns:
            int: Batches per epoch
        """
        if self.temporal_chunk_list is not None:
           return len(self.temporal_chunk_list) // self.temporal_batch_size

        return len(self.list_IDs) // self.batch_size
    
    def _update_temporal_batch_size(self):
        if self.temporal_chunk_list is not None:
            self.temporal_chunk_size = len(self.temporal_chunk_list[0])
            self.temporal_batch_size = self.batch_size // self.temporal_chunk_size

    def __getitem__(self, index):
        """Generate one batch of data.

        Args:
            index (int): Frame index

        Returns:
            Tuple[np.ndarray, np.ndarray]: One batch of data
                X (np.ndarray): Input volume
                y (np.ndarray): Target
        """
        if self.temporal_chunk_list is not None:
            i = index * self.temporal_batch_size
            indexes = self.indexes[i : i + self.temporal_batch_size]
            list_IDs_temp = list(np.concatenate([self.temporal_chunk_list[k] for k in indexes], axis=0))
            
        else: 
            indexes = self.indexes[index * self.batch_size : (index + 1) * self.batch_size]
            list_IDs_temp = [self.list_IDs[k] for k in indexes]
        # Generate data
        X, y = self.__data_generation(list_IDs_temp)
        return X, y

    def on_epoch_end(self):
        """Update indexes after each epoch."""
        if self.temporal_chunk_list is not None:
            self.indexes = np.arange(len(self.temporal_chunk_list))
        else:    
            self.indexes = np.arange(len(self.list_IDs))
            
        if self.shuffle:
            np.random.shuffle(self.indexes)

    def rot90(self, X):
        """Rotate X by 90 degrees CCW.

        Args:
            X (np.ndarray): Image volume or grid

        Returns:
            X (np.ndarray): Rotated image volume or grid
        """
        X = np.transpose(X, [1, 0, 2, 3])
        X = X[:, ::-1, :, :]
        return X

    def mirror(self, X, y_3d, X_grid):
        # Flip the image and x coordinates about the x axis
        X = X[:, ::-1, ...]
        X_grid = X_grid[:, ::-1, ...]

        # Flip the left and right keypoints.
        temp = y_3d[..., self.left_keypoints].copy()
        y_3d[..., self.left_keypoints] = y_3d[..., self.right_keypoints]
        y_3d[..., self.right_keypoints] = temp
        return X, y_3d, X_grid

    def rot180(self, X):
        """Rotate X by 180 degrees.

        Args:
            X (np.ndarray): Image volume or grid

        Returns:
            X (np.ndarray): Rotated image volume or grid
        """
        X = X[::-1, ::-1, :, :]
        return X

    def random_rotate(self, X, y_3d, aux=None):
        """Rotate each sample by 0, 90, 180, or 270 degrees.

        Args:
            X (np.ndarray): Image volumes
            y_3d (np.ndarray): 3D grid coordinates (AVG) or training target volumes (MAX)
            aux (np.ndarray or None): Populated in MAX+AVG mode with the training target volumes
        Returns:
            X (np.ndarray): Rotated image volumes
            y_3d (np.ndarray): Rotated 3D grid coordinates (AVG) or training target volumes (MAX)
        """
        # if use temporal, rotation augmentations have to be consistent within each temporal chunks
        # for example, assume the temporal chunk size to be 2, each batch contains exactly 2 chunks
        # then, only 2 different rotation schemes need to be generated
        rots = np.random.choice(np.arange(4), self.temporal_batch_size)
        for i in range(0, self.temporal_chunk_size, self.batch_size):
            if rots[i] == 0:
                pass
            elif rots[i] == 1:
                # Rotate180
                for j in range(i, i+self.temporal_chunk_size):
                    X[j], y_3d[j] = self.rot180(X[j]), self.rot180(y_3d[j])
                    if aux is not None:
                        aux[j] = self.rot180(aux[j])
            elif rots[i] == 2:
                # Rotate90
                for j in range(i, i+self.temporal_chunk_size):
                    X[j], y_3d[j] = self.rot90(X[j]), self.rot90(y_3d[j])
                    if aux is not None:
                        aux[j] = self.rot90(aux[j])
            elif rots[i] == 3:
                # Rotate -90/270
                for j in range(i, i+self.temporal_chunk_size):
                    X[j], y_3d[j] = self.rot180(self.rot90(X[j])), self.rot180(self.rot90(y_3d[j]))
                    if aux is not None:
                        aux[j] = self.rot180(self.rot90(aux[j]))
        if aux is not None:
            return X, y_3d, aux
        return X, y_3d

    def visualize(self, original, augmented):
        """Plots example image after augmentation

        Args:
            original (np.ndarray): image before augmentation
            augmented (np.ndarray): image after augmentation.
        """
        import matplotlib.pyplot as plt

        fig = plt.figure()
        plt.subplot(1, 2, 1)
        plt.title("Original image")
        plt.imshow(original)

        plt.subplot(1, 2, 2)
        plt.title("Augmented image")
        plt.imshow(augmented)
        plt.show()
        input("Press Enter to continue...")

    def do_augmentation(self, X, X_grid, y_3d, aux=None):
        """Applies augmentation

        Args:
            X (np.ndarray): image volumes
            X_grid (np.ndarray): 3D grid coordinates
            y_3d (np.ndarray): training targets
            aux (np.ndarray or None): additional target volumes if using MAX+AVG mode

        Returns:
            X (np.ndarray): Augemented image volumes
            X_grid (np.ndarray): 3D grid coordinates
            y_3d (np.ndarray): Training targets
        """
        if self.rotation:
            if self.expval:
                # First make X_grid 3d
                X_grid = np.reshape(
                    X_grid,
                    (self.batch_size, self.nvox, self.nvox, self.nvox, 3),
                )
                if aux is not None:
                    X, X_grid, aux = self.random_rotate(X.copy(), X_grid.copy(), aux.copy())
                else:
                    X, X_grid = self.random_rotate(X.copy(), X_grid.copy())
                # Need to reshape back to raveled version
                X_grid = np.reshape(X_grid, (self.batch_size, -1, 3))
            else:
                X, y_3d = self.random_rotate(X.copy(), y_3d.copy())

        if self.augment_continuous_rotation and aux is None:
            if self.expval:
                # First make X_grid 3d
                X_grid = np.reshape(
                    X_grid,
                    (self.batch_size, self.nvox, self.nvox, self.nvox, 3),
                )
                X, X_grid = random_continuous_rotation(
                    X.copy(), X_grid.copy(), self.rotation_val
                )
                # Need to reshape back to raveled version
                X_grid = np.reshape(X_grid, (self.batch_size, -1, 3))
            else:
                X, y_3d = random_continuous_rotation(
                    X.copy(), y_3d.copy(), self.rotation_val
                )

        if self.augment_hue and self.chan_num == 3:
            for n_cam in range(int(X.shape[-1] / self.chan_num)):
                channel_ids = np.arange(
                    n_cam * self.chan_num,
                    n_cam * self.chan_num + self.chan_num,
                )
                X[..., channel_ids] = tf.image.random_hue(
                    X[..., channel_ids], self.hue_val
                )
        elif self.augment_hue:
            warnings.warn(
                "Trying to augment hue with an image that is not RGB. Skipping."
            )

        if self.augment_brightness:
            for n_cam in range(int(X.shape[-1] / self.chan_num)):
                channel_ids = np.arange(
                    n_cam * self.chan_num,
                    n_cam * self.chan_num + self.chan_num,
                )
                X[..., channel_ids] = tf.image.random_brightness(
                    X[..., channel_ids], self.bright_val
                )

        if self.mirror_augmentation and self.expval and aux is None:
            if np.random.rand() > 0.5:
                X_grid = np.reshape(
                    X_grid,
                    (self.batch_size, self.nvox, self.nvox, self.nvox, 3),
                )
                # Flip the image and the symmetric keypoints
                X, y_3d, X_grid = self.mirror(X.copy(), y_3d.copy(), X_grid.copy())
                X_grid = np.reshape(X_grid, (self.batch_size, -1, 3))
        else:
            pass
            ##TODO: implement mirror augmentation for max and avg+max modes

        return X, X_grid, y_3d, aux

    def do_random(self, X):
        """Randomly re-order camera views

        Args:
            X (np.ndarray): image volumes

        Returns:
            X (np.ndarray): Shuffled image volumes
        """

        if self.random:
            X = X.reshape((*X.shape[:4], self.chan_num, -1), order="F")
            X = X[..., np.random.permutation(X.shape[-1])]
            X = X.reshape((*X.shape[:4], -1), order="F")

        if self.n_rand_views is not None:
            # Select a set of cameras randomly with replacement.
            X = X.reshape((*X.shape[:4], self.chan_num, -1), order="F")
            if self.replace:
                X = X[..., np.random.randint(X.shape[-1], size=(self.n_rand_views,))]
            else:
                if not self.random:
                    raise Exception(
                        "For replace=False for n_rand_views, random must be turned on"
                    ) 
                X = X[..., :self.n_rand_views]
            X = X.reshape((*X.shape[:4], -1), order="F")

        return X

    def get_max_gt_ind(self, X_grid, y_3d):
        """Uses the gt label position to find the index of the voxel corresponding to it.
        Used for heatmap regularization.
        """

        diff = np.sum(
            (X_grid[:, :, :, np.newaxis] - y_3d[:, np.newaxis, :, :]) ** 2, axis=2
        )
        inds = np.argmin(diff, axis=1)
        grid_d = int(np.round(X_grid.shape[1] ** (1 / 3)))
        inds = np.unravel_index(inds, (grid_d, grid_d, grid_d))
        return np.stack(inds, axis=1)

    def return_multi_inputs_targets(self, X, X_grid, y_3d, aux):
        # # adjust inputs & outputs according to targeted losses
        return_input, return_target = [X], {'final_output': y_3d}
        if not self.expval:
            return return_input, return_target

        return_input.append(X_grid)
        final_output_count = 1

        
        if self.temporal_chunk_list is not None:
            return_target[f'final_output_{final_output_count}'] = y_3d
            final_output_count += 1
        
        if self.separation_loss:
            return_target[f'final_output_{final_output_count}'] = y_3d
            final_output_count += 1
        
        if self.symmetry_loss:
            return_target[f'final_output_{final_output_count}'] = y_3d
            final_output_count += 1
        
        if self.heatmap_reg:
            return_input.append(self.get_max_gt_ind(X_grid, y_3d))
            return_target['heatmap_output'] = self.heatmap_reg_coeff*np.ones((self.batch_size, y_3d.shape[-1]), dtype='float32')

        if self.gaussian_reg:
            return_target['gaussian_output'] = y_3d

        if aux is not None:
            return_target['normed_map'] = aux 
        
        return return_input, return_target

    def __data_generation(self, list_IDs_temp):
        """Generate data containing batch_size samples.
        X : (n_samples, *dim, n_channels)

        Args:
            list_IDs_temp (List): List of experiment Ids

        Returns:
            Tuple: Batch_size training samples
                X: Input volumes
                y_3d: Targets
        Raises:
            Exception: For replace=False for n_rand_views, random must be turned on.
        """
        # Initialization

        X = np.zeros((self.batch_size, *self.data.shape[1:]))
        y_3d = np.zeros((self.batch_size, *self.labels.shape[1:]))

        # Only used for AVG mode
        if self.expval:
            X_grid = np.zeros((self.batch_size, *self.xgrid.shape[1:]))
        else:
            X_grid = None

        # Only used for AVG+MAX mode
        if self.aux_labels is not None:
            aux = np.zeros((*X.shape[:4], y_3d.shape[-1]))
        else:
            aux = None

        for i, ID in enumerate(list_IDs_temp):
            X[i] = self.data[ID].copy()
            y_3d[i] = self.labels[ID]
            if self.expval:
                X_grid[i] = self.xgrid[ID]
            if aux is not None:
                aux[i] = self.aux_labels[ID]
        X, X_grid, y_3d, aux = self.do_augmentation(X, X_grid, y_3d, aux)
        # Randomly re-order, if desired
        X = self.do_random(X)
        
        # # adjust inputs & outputs according to targeted losses        
        return self.return_multi_inputs_targets(X, X_grid, y_3d, aux)


class DataGenerator_3Dconv_npy(DataGenerator_3Dconv_frommem):
    """Generates 3d conv data from npy files.

    Attributes:
    augment_brightness (bool): If True, applies brightness augmentation
    augment_continuous_rotation (bool): If True, applies rotation augmentation in increments smaller than 90 degrees
    augment_hue (bool): If True, applies hue augmentation
    batch_size (int): Batch size
    bright_val (float): Brightness augmentation range (-bright_val, bright_val), as fraction of raw image brightness
    chan_num (int): Number of input channels
    labels_3d (Dict): training targets
    expval (bool): If True, crafts input for an AVG network
    hue_val (float): Hue augmentation range (-hue_val, hue_val), as fraction of raw image hue range
    indexes (np.ndarray): Sample indices used for batch generation
    list_IDs (List): List of sampleIDs
    nvox (int): Number of voxels in each grid dimension
    random (bool): If True, shuffles camera order for each batch
    rotation (bool): If True, applies rotation augmentation in 90 degree increments
    rotation_val (float): Range of angles used for continuous rotation augmentation
    shuffle (bool): If True, shuffle the samples before each epoch
    var_reg (bool): If True, returns input used for variance regularization
    n_rand_views (int): Number of reviews to sample randomly from the full set
    replace (bool): If True, samples n_rand_views with replacement
    imdir (Text): Name of image volume npy subfolder
    griddir (Text): Name of grid volumw npy subfolder
    mono (bool): If True, return monochrome image volumes
    sigma (float): For MAX network, size of target Gaussian (mm)
    cam1 (bool): If True, prepares input for training a single camea network
    prefeat (bool): If True, prepares input for a network performing volume feature extraction before fusion
    npydir (Dict): path to each npy volume folder for each recording (i.e. experiment)
    """

    def __init__(
        self,
        list_IDs,
        labels_3d,
        npydir,
        batch_size,
        imdir="image_volumes",
        griddir="grid_volumes",
        prefeat=False,        
        mono=False,
        cam1=False,    
        sigma=10,
        **kwargs
    ):
        """Generates 3d conv data from npy files.

        Args:
            list_IDs (List): List of sampleIDs
            labels_3d (Dict): training targets
            npydir (Dict): path to each npy volume folder for each recording (i.e. experiment)
            batch_size (int): Batch size
            imdir (Text, optional): Name of image volume npy subfolder
            griddir (Text, optional): Name of grid volumw npy subfolder
            mono (bool, optional): If True, return monochrome image volumes
            cam1 (bool, optional): If True, prepares input for training a single camea network
            prefeat (bool, optional): If True, prepares input for a network performing volume feature extraction before fusion
            sigma (float, optional): For MAX network, size of target Gaussian (mm)
        """
        super(DataGenerator_3Dconv_npy, self).__init__(
            list_IDs=list_IDs,
            data=None,
            labels=None,
            batch_size=batch_size,
            **kwargs
        )
        # self.list_IDs = list_IDs
        self.labels_3d = labels_3d
        self.npydir = npydir
        self.griddir = griddir
        self.imdir = imdir
        self.mono = mono
        self.cam1 = cam1
        #self.replace = replace
        self.prefeat = prefeat
        self.sigma = sigma

    def __getitem__(self, index):
        """Generate one batch of data.

        Args:
            index (int): Frame index

        Returns:
            Tuple[np.ndarray, np.ndarray]: One batch of data
                X (np.ndarray): Input volume
                y (np.ndarray): Target
        """
        if self.temporal_chunk_list is not None:
            i = index * self.temporal_batch_size
            indexes = self.indexes[i : i + self.temporal_batch_size]
            indexes = list(np.concatenate([self.temporal_chunk_list[k] for k in indexes], axis=0))
            list_IDs_temp = [self.list_IDs[k] for k in indexes]
        else: 
            indexes = self.indexes[index * self.batch_size : (index + 1) * self.batch_size]
            list_IDs_temp = [self.list_IDs[k] for k in indexes]
        # Generate data
        X, y = self.__data_generation(list_IDs_temp)
        return X, y

    def __data_generation(self, list_IDs_temp):
        """Generate data containing batch_size samples.
        X : (n_samples, *dim, n_channels)

        Args:
            list_IDs_temp (List): List of experiment Ids

        Returns:
            Tuple: Batch_size training samples
                X: Input volumes
                y_3d or y_3d_max: Targets
        Raises:
            Exception: For replace=False for n_rand_views, random must be turned on.
        """

        # Initialization

        X = []
        y_3d = []
        X_grid = []

        for i, ID in enumerate(list_IDs_temp):
            # Need to look up the experiment ID to get the correct directory
            IDkey = ID.split("_")
            eID = int(IDkey[0])
            sID = IDkey[1]

            X.append(
                np.load(
                    os.path.join(self.npydir[eID], self.imdir, "0_" + sID + ".npy")
                ).astype("float32")
            )

            y_3d.append(self.labels_3d[ID])
            X_grid.append(
                np.load(
                    os.path.join(self.npydir[eID], self.griddir, "0_" + sID + ".npy")
                )
            )

        X = np.stack(X)
        y_3d = np.stack(y_3d)

        X_grid = np.stack(X_grid)
        if not self.expval:
            y_3d_max = np.zeros(
                (self.batch_size, self.nvox, self.nvox, self.nvox, y_3d.shape[-1])
            )

        if not self.expval:
            X_grid = np.reshape(X_grid, (-1, self.nvox, self.nvox, self.nvox, 3))
            for gridi in range(X_grid.shape[0]):
                x_coord_3d = X_grid[gridi, :, :, :, 0]
                y_coord_3d = X_grid[gridi, :, :, :, 1]
                z_coord_3d = X_grid[gridi, :, :, :, 2]
                for j in range(y_3d_max.shape[-1]):
                    y_3d_max[gridi, :, :, :, j] = np.exp(
                        -(
                            (y_coord_3d - y_3d[gridi, 1, j]) ** 2
                            + (x_coord_3d - y_3d[gridi, 0, j]) ** 2
                            + (z_coord_3d - y_3d[gridi, 2, j]) ** 2
                        )
                        / (2 * self.sigma ** 2)
                    )

        if self.mono and self.chan_num == 3:
            # Convert from RGB to mono using the skimage formula. Drop the duplicated frames.
            # Reshape so RGB can be processed easily.
            X = np.reshape(
                X,
                (
                    X.shape[0],
                    X.shape[1],
                    X.shape[2],
                    X.shape[3],
                    self.chan_num,
                    -1,
                ),
                order="F",
            )
            X = (
                X[:, :, :, :, 0] * 0.2125
                + X[:, :, :, :, 1] * 0.7154
                + X[:, :, :, :, 2] * 0.0721
            )

        ncam = int(X.shape[-1] // self.chan_num)

        X, X_grid, y_3d, aux = self.do_augmentation(X, X_grid, y_3d)

        # Randomly re-order, if desired
        X = self.do_random(X)

        if self.cam1:
            # collapse the cameras to the batch dimensions.
            X = np.reshape(
                X,
                (X.shape[0], X.shape[1], X.shape[2], X.shape[3], self.chan_num, -1),
                order="F",
            )
            X = np.transpose(X, [0, 5, 1, 2, 3, 4])
            X = np.reshape(X, (-1, X.shape[2], X.shape[3], X.shape[4], X.shape[5]))
            if self.expval:
                y_3d = np.tile(y_3d, [ncam, 1, 1])
                X_grid = np.tile(X_grid, [ncam, 1, 1])
            else:
                y_3d = np.tile(y_3d, [ncam, 1, 1, 1, 1])

        # TODO: confirm that this line is redundant
        # X = processing.preprocess_3d(X) 

        # # adjust inputs & outputs according to targeted losses
        return self.return_multi_inputs_targets(X, X_grid, y_3d, aux)