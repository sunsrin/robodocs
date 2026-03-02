# Copyright (c) 2022-2026 Littleton Robotics
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

import math
from pathlib import Path
import time
from typing import Dict, List, Optional, Tuple, Union

import coremltools
import cv2
import numpy as np
from config.config import ConfigStore
from PIL import Image
from vision_types import ObjDetectObservation


class ObjectDetector:
    def __init__(self) -> None:
        raise NotImplementedError

    def detect(self, image: cv2.Mat, config: ConfigStore) -> List[ObjDetectObservation]:
        raise NotImplementedError


class CoreMLObjectDetector(ObjectDetector):
    _model: Union[coremltools.models.MLModel, None] = None

    def __init__(self) -> None:
        pass

    def detect(self, image: cv2.Mat, config: ConfigStore) -> List[ObjDetectObservation]:
        # Load CoreML model
        if self._model == None:
            print("Loading object detection model")
            self._model = coremltools.models.MLModel(config.local_config.obj_detect_model)
            print("Finished loading object detection model")

        # Create scaled frame for model
        if len(image.shape) == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        image_scaled = np.zeros((640, 640, 3), dtype=np.uint8)
        scaled_height = int(640 / (image.shape[1] / image.shape[0]))
        bar_height = int((640 - scaled_height) / 2)
        image_scaled[bar_height: bar_height + scaled_height, 0:640] = cv2.resize(image, (640, scaled_height))

        # Run CoreML model
        image_coreml = Image.fromarray(image_scaled)
        for _ in range(4):
            prediction = self._model.predict({"image": image_coreml})

        observations: List[ObjDetectObservation] = []
        for coordinates, confidence in zip(prediction["coordinates"], prediction["confidence"]):
            obj_class = max(range(len(confidence)), key=confidence.__getitem__)
            confidence = float(confidence[obj_class])
            x = coordinates[0] * image.shape[1]
            y = ((coordinates[1] * 640 - bar_height) / scaled_height) * image.shape[0]
            width = coordinates[2] * image.shape[1]
            height = coordinates[3] / (scaled_height / 640) * image.shape[0]

            corners = np.array(
                [
                    [x - width / 2, y - height / 2],
                    [x + width / 2, y - height / 2],
                    [x - width / 2, y + height / 2],
                    [x + width / 2, y + height / 2],
                ]
            )
            corners_undistorted = cv2.undistortPoints(
                corners,
                config.local_config.camera_matrix,
                config.local_config.distortion_coefficients,
                None,
                config.local_config.camera_matrix,
            )

            corner_angles = np.zeros((4, 2))
            for index, corner in enumerate(corners_undistorted):
                vec = np.linalg.inv(config.local_config.camera_matrix).dot(np.array([corner[0][0], corner[0][1], 1]).T)
                corner_angles[index][0] = math.atan(vec[0])
                corner_angles[index][1] = math.atan(vec[1])

            observations.append(ObjDetectObservation(obj_class, confidence, corner_angles, corners))

        return observations


class CoreMLSlicedObjectDetector(ObjectDetector):
    edge_filter_width: int = 100
    imgsz: Tuple = (960, 1280, 3)
    size: int = 640
    overlap: int = 128
    nchan: int = 3
    run_nms: bool = True
    run_edge_filter: bool = True
    nms_iou_thresh: float = 0.80
    nms_conf_thresh: float = 0.05

    # (Row, Column, H, W)
    slice4_bbox = [[0, int(overlap/2-1), size, size],
                   [0, int(imgsz[1]/2 - overlap/2-1), size, size],
                   [imgsz[0]-size-1, int(overlap/2-1), size, size],
                   [imgsz[0]-size-1, int(imgsz[1]/2-overlap/2-1), size, size]]

    # Indicate which edges to filter for overlap (Left, Right, Top, Bottom)
    slice4_edge_overlap = [[0, 1, 0, 1],
                           [1, 0, 0, 1],
                           [0, 1, 1, 0],
                           [1, 0, 1, 0]]

    # (Row, Column, H, W)
    slice6_bbox = [[0, 0, size, size],
                   [0, int(imgsz[1]/2 - size/2), size, size],
                   [0, int(imgsz[1] - size), size, size],
                   [imgsz[0]-size-1, 0, size, size],
                   [imgsz[0]-size-1, int(imgsz[1]/2 - size/2), size, size],
                   [imgsz[0]-size-1, int(imgsz[1] - size), size, size]]

    # Indicate which edges to filter for overlap (Left, Right, Top, Bottom)
    slice6_edge_overlap = [[0, 1, 0, 1],
                           [1, 1, 0, 1],
                           [1, 0, 0, 1],
                           [0, 1, 1, 0],
                           [1, 1, 1, 0],
                           [1, 0, 1, 0]]
    _model: Union[coremltools.models.MLModel, None] = None

    def __init__(self, model_path: Optional[Path] = None):
        """ Constructor

        Parameters
        ----------
        model_path : Path
            Path to a detection model file. The constructor will attempt to
            load the model (PyTorch/Ultralytics by default) via
            :meth:`_load_model`.

        Side effects
        -----------
        - Loads the model and stores it on ``self._model``.
        - Initializes ``self.timing`` for profiling stages.
        - Selects default slice bounding boxes and edge overlap map.
        """
        assert CoreMLSlicedObjectDetector.overlap % 2 == 0 and CoreMLSlicedObjectDetector.size % 2 == 0, "Overlap and size must be even"
        self.coreml = False

        if model_path is not None:
            coreml = True if (model_path.suffix == ".mlpackage") else False
            self._model_path: Path = model_path
            self._load_model(model_path, coreml=coreml)

        self.timing: Dict[str, float] = {}
        self.slices = CoreMLSlicedObjectDetector.slice6_bbox
        self.edge_overlap = CoreMLSlicedObjectDetector.slice6_edge_overlap

    def _load_model(self, model_path: Path, coreml: bool = False):
        """Load the object detection model from disk.

        Parameters
        ----------
        model_path : Path
            Filesystem path to the model file to load.
        coreml : bool, optional
            If True attempt to load a CoreML model via ``coremltools``.
            If False (default) the function will try to import and use
            the Ultralytics ``YOLO`` wrapper.

        Raises
        ------
        RuntimeError
            If the required backend library (``coremltools`` or
            ``ultralytics``) cannot be imported.
        """

        print("Loading batched object detection model")
        if coreml:
            try:
                import coremltools
            except:
                raise RuntimeError("Unable to load coreml tools")
            self._model = coremltools.models.MLModel(str(model_path))
            self.coreml = True
        else:
            try:
                from ultralytics import YOLO
            except:
                raise RuntimeError("Unable to load YOLO")

            self._model = YOLO(model_path, task="detect")
            self.coreml = False

        print("Finished loading batched object detection model")

    def _slice_image(self, image: cv2.Mat) -> Tuple[List[List[int]], np.ndarray]:
        """Split an RGB image into the configured overlapping tiles.

        The detector operates on fixed-size tiles; this helper extracts the
        tiles defined in ``self.slices`` and returns both the slice
        bounding boxes and a batch array formatted for model input.

        Parameters
        ----------
        image : cv2.Mat
            Input image (RGB). Expected shape matches ``self.imgsz``.

        Returns
        -------
        Tuple[List[List[int]], np.ndarray]
            A tuple containing the list of slice bounding boxes
            (each entry is [row, col, height, width]) and a numpy array
            of shape (N, 3, H, W) with the tile pixels arranged as
            channel-first float32 values.

        Raises
        ------
        AssertionError
            If ``image.shape`` does not match ``self.imgsz``.
        """

        assert image.shape == self.imgsz, "Unsupported image size"

        img_batch = np.zeros((len(self.slices), 3, self.size, self.size), dtype=np.float32)
        for idx, (x, y, h, w) in enumerate(self.slices):
            # print(f"Preparing slice: x={x} y={y} h ={h} w={w}")
            img_batch[idx] = np.transpose(image[x:(x+h), y:(y+w)], (2, 0, 1))

        return (self.slices, img_batch)

    def _filter_edge(self, dets: List[np.ndarray]) -> List[np.ndarray]:
        """Filter detections that fall within the configured edge overlap margins.

        Many of the tiles overlap; to avoid duplicate detections near tile
        borders we drop boxes that lie within ``self.edge_filter_width`` of
        the configured edges for a given slice. The per-slice edge mask is
        read from ``self.edge_overlap`` and uses the order
        ``[left, right, top, bottom]``.

        Parameters
        ----------
        dets : List[np.ndarray]
            A list with one array per slice. Each array is expected to be
            Nx6 with columns (x1, y1, x2, y2, conf, cls) in slice-local
            coordinates.

        Returns
        -------
        List[np.ndarray]
            The same list of arrays but with boxes near configured edges
            removed according to the overlap rules.
        """

        for idx, slice_dets in enumerate(dets):

            keep = np.ones(slice_dets.shape[0], dtype=bool)
            edges = self.edge_overlap[idx]

            x1 = dets[idx][:, 0]
            y1 = dets[idx][:, 1]
            x2 = dets[idx][:, 2]
            y2 = dets[idx][:, 3]

            # Left: drop boxes that start within `edge_filter` pixels of left edge
            if edges[0]:
                keep &= (x1 >= self.edge_filter_width)

            # Right: drop boxes that end within `edge_filter` pixels of right edge
            if edges[1]:
                keep &= (x2 <= (self.size - self.edge_filter_width))

            # Top: drop boxes that start within `edge_filter` pixels of top edge
            if edges[2]:
                keep &= (y1 >= self.edge_filter_width)

            # Bottom: drop boxes that end within `edge_filter` pixels of bottom edge
            if edges[3]:
                keep &= (y2 <= (self.size - self.edge_filter_width))

            dets[idx] = dets[idx][keep]

        return dets

    def _merge_dets(self, slice_bbox: List[List[int]], dets: List[np.ndarray]) -> np.ndarray:
        """Merge per-slice detections back into full-image coordinates.

        This function optionally applies the edge filter to per-slice
        detections, concatenates all slice detections, offsets their box
        coordinates to the full-image coordinate system and finally runs
        class-aware non-maximum suppression (NMS) if enabled.

        Parameters
        ----------
        slice_bbox : List[List[int]]
            List of slice bounding boxes in image coordinates. Each entry
            is ``[row, col, height, width]`` and is used to compute the
            offset to apply to each slice's detections.
        dets : List[np.ndarray]
            Per-slice detection arrays (one per slice) with rows of the
            form ``[x1, y1, x2, y2, conf, cls]`` in slice-local pixels.

        Returns
        -------
        np.ndarray
            A single (M,6) array of merged detections in full-image
            coordinates. If no detections are present an empty array with
            shape (0,6) or a zero-row array is returned.
        """

        if CoreMLSlicedObjectDetector.run_edge_filter == True:
            dets = self._filter_edge(dets)

        det_count = [d.shape[0] for d in dets]
        total_dets = np.sum(det_count, dtype=np.int32)
        merged_dets = np.zeros((total_dets, 6), dtype=np.float32)
        det_idx = 0
        for slice_idx, slice in enumerate(slice_bbox):
            offset = np.array([slice[1], slice[0], slice[1], slice[0]])
            merged_dets[det_idx:det_idx + det_count[slice_idx]] = dets[slice_idx]
            merged_dets[det_idx:det_idx + det_count[slice_idx], 0:4] += offset
            det_idx += det_count[slice_idx]

        # run class-aware non-max suppression and confidence thresholding
        if total_dets == 0:
            return merged_dets

        if CoreMLSlicedObjectDetector.run_nms == True:
            merged_dets = self._nms(merged_dets, iou_thresh=CoreMLSlicedObjectDetector.nms_iou_thresh,
                                    conf_thresh=CoreMLSlicedObjectDetector.nms_conf_thresh)

        return merged_dets

    def _nms(self, dets: np.ndarray, iou_thresh: float = 0.45, conf_thresh: float = 0.10) -> np.ndarray:
        """Perform class-aware non-maximum suppression (NMS).

        Parameters
        ----------
        dets : np.ndarray
            Array of detections with shape (N,6) where each row is
            ``x1, y1, x2, y2, conf, cls``. Coordinates may be in pixels.
        iou_thresh : float, optional
            IoU threshold used to suppress overlapping boxes of the same
            class (default 0.45). Boxes with IoU > iou_thresh are
            suppressed.
        conf_thresh : float, optional
            Minimum confidence required to keep a detection (default
            0.10). Detections with ``conf < conf_thresh`` are filtered
            before NMS.

        Returns
        -------
        np.ndarray
            The filtered detections in the same (x1,y1,x2,y2,conf,cls)
            format.
        """
        if dets is None or len(dets) == 0:
            return dets

        dets = np.asarray(dets).astype(np.float32)

        # filter by confidence
        keep_mask = dets[:, 4] >= conf_thresh
        dets = dets[keep_mask]
        if dets.shape[0] == 0:
            return dets

        # ensure box format and sane coords
        x1 = dets[:, 0].astype(float)
        y1 = dets[:, 1].astype(float)
        x2 = dets[:, 2].astype(float)
        y2 = dets[:, 3].astype(float)
        scores = dets[:, 4].astype(float)
        classes = dets[:, 5].astype(int)

        # clamp boxes to valid ranges (swap if necessary)
        x1c = np.minimum(x1, x2)
        x2c = np.maximum(x1, x2)
        y1c = np.minimum(y1, y2)
        y2c = np.maximum(y1, y2)

        out_boxes = []

        unique_classes = np.unique(classes)
        for cls in unique_classes:
            inds = np.where(classes == cls)[0]
            if inds.size == 0:
                continue

            cls_x1 = x1c[inds]
            cls_y1 = y1c[inds]
            cls_x2 = x2c[inds]
            cls_y2 = y2c[inds]
            cls_scores = scores[inds]
            cls_dets = dets[inds]

            # sort by score desc
            order = cls_scores.argsort()[::-1]

            keep = []
            while order.size > 0:
                i = order[0]
                keep.append(i)

                xx1 = np.maximum(cls_x1[i], cls_x1[order[1:]])
                yy1 = np.maximum(cls_y1[i], cls_y1[order[1:]])
                xx2 = np.minimum(cls_x2[i], cls_x2[order[1:]])
                yy2 = np.minimum(cls_y2[i], cls_y2[order[1:]])

                w = np.maximum(0.0, xx2 - xx1 + 1.0)
                h = np.maximum(0.0, yy2 - yy1 + 1.0)
                inter = w * h

                area_i = (cls_x2[i] - cls_x1[i] + 1.0) * (cls_y2[i] - cls_y1[i] + 1.0)
                area_others = (cls_x2[order[1:]] - cls_x1[order[1:]] + 1.0) * \
                    (cls_y2[order[1:]] - cls_y1[order[1:]] + 1.0)

                union = area_i + area_others - inter
                iou = inter / np.maximum(union, 1e-6)

                # keep boxes with IoU <= threshold
                inds_to_keep = np.where(iou <= iou_thresh)[0]
                order = order[inds_to_keep + 1]

            # append kept boxes for this class
            out_boxes.extend(cls_dets[keep].tolist())

        if len(out_boxes) == 0:
            return np.zeros((0, 6), dtype=np.float32)

        out_arr = np.asarray(out_boxes, dtype=np.float32)
        return out_arr

    def detect(self, image: cv2.Mat, config: ConfigStore) -> List[ObjDetectObservation]:
        """Run batched object detection on an input BGR image.

        The method performs the following high-level steps:
        1. Convert the input from BGR to RGB and split it into overlapping
        tiles via :meth:`_slice_image`.
        2. Run the model on the tile batch and collect per-tile detections.
        3. Merge per-tile detections into full-image coordinates using
        :meth:`_merge_dets` and (optionally) visualize results.

        Parameters
        ----------
        image : cv2.Mat
            Input image in BGR color order. Expected shape is ``(960, 1280, 3)``.
        config : ConfigStore
            Configuration store containing camera parameters and model path.

        Returns
        -------
        List[ObjDetectObservation]
            List of object detection observations with undistorted corner
            angles and pixel coordinates for each detected object.
        """
        # -- Check configuration
        if self._model == None:
            model_path = Path(config.local_config.obj_detect_model)
            coreml = True if (model_path.suffix == ".mlpackage") else False
            self._load_model(model_path, coreml=coreml)

        # -- Slice image
        start_time = time.time()
        brg_image = image
        image = brg_image
        slice_bbox, img_batch = self._slice_image(image)
        img_batch /= 255.0
        nbatches = len(slice_bbox)
        self.timing["slice"] = time.time() - start_time

        # -- Run inference
        start_time = time.time()

        prediction = {"output": np.zeros((6, 300, 6))}
        if self.coreml:
            for i in range(nbatches):
                img = np.expand_dims(img_batch[i], axis=0)
                prediction_single = self._model.predict({"input": img})
                prediction["output"][i] = prediction_single["output"][0]
        else:
            prediction = self._model(torch.tensor(img_batch))

        self.timing["inference"] = time.time() - start_time

        # -- Merge detections
        start_time = time.time()
        if isinstance(prediction, list) and hasattr(prediction[0], 'boxes'):
            dets = [b.boxes.data.cpu().detach().numpy() for b in prediction]
        else:
            key = next(iter(prediction), None)
            dets_arr = prediction[key]
            dets = [dets_arr[i, :, :] for i in range(dets_arr.shape[0])]

        if False:
            for i in range(nbatches):
                img = (np.transpose(img_batch[i], (1, 2, 0))[..., ::-1]*255).astype(np.uint8)
                self._visualize_detections(img, dets[i], wait=False,
                                           out_path=f"/Users/frc6328/tmp/ws/img_slice/output/{i}.png")

        merged_dets = self._merge_dets(slice_bbox, dets)
        self.timing["merge"] = time.time() - start_time

        if False:
            annoted_img = self._visualize_detections(
                brg_image, merged_dets,  title="Merged Slice Inference", out_path="output/final.png", wait=False)

        observations: List[ObjDetectObservation] = []
        for det in merged_dets:
            obj_class = int(det[5])
            confidence = det[4]
            corners = np.array([
                [det[0], det[1]],
                [det[2], det[1]],
                [det[0], det[3]],
                [det[2], det[3]]
            ])

            corners_undistorted = cv2.undistortPoints(
                corners,
                config.local_config.camera_matrix,
                config.local_config.distortion_coefficients,
                None,
                config.local_config.camera_matrix,
            )

            corner_angles = np.zeros((4, 2))
            for index, corner in enumerate(corners_undistorted):
                vec = np.linalg.inv(config.local_config.camera_matrix).dot(np.array([corner[0][0], corner[0][1], 1]).T)
                corner_angles[index][0] = math.atan(vec[0])
                corner_angles[index][1] = math.atan(vec[1])

            observations.append(ObjDetectObservation(obj_class, confidence, corner_angles, corners))

        # print(self.timing)
        return observations

    def _visualize_slices(self, slice_bbox: List, imgs: np.ndarray, out_path: str = None, wait: bool = True):
        """Compose and display (or save) image slices arranged into a canvas.

        Parameters
        ----------
        slice_bbox : List
            List of slice bounding boxes in the form ``[row, col, height, width]``.
        imgs : np.ndarray
            Array shaped ``(N, 3, H, W)`` containing tile pixels in channel-first
            order.
        out_path : str, optional
            If provided, the composed canvas will be written to this path.
        wait : bool, optional
            When True (default) display the image window and wait for a
            keypress before returning.

        Returns
        -------
        np.ndarray
            The composed canvas image (H, W, 3) in uint8 BGR order.
        """

        # infer canvas size from bounding boxes
        max_row = max(x + h for (x, y, h, w) in slice_bbox)
        max_col = max(y + w for (x, y, h, w) in slice_bbox)
        canvas = np.zeros((max_row, max_col, 3), dtype=np.uint8)

        for idx, (x, y, h, w) in enumerate(slice_bbox):
            # convert from (3,H,W) -> (H,W,3)
            sl = imgs[idx]
            sl_img = np.transpose(sl, (1, 2, 0))
            # ensure uint8 for display
            if sl_img.dtype != np.uint8:
                sl_img = np.clip(sl_img, 0, 255).astype(np.uint8)

            # blend into canvas (handles overlaps)
            roi = canvas[x:(x + h), y:(y + w)]
            if roi.shape == sl_img.shape and roi.sum() != 0:
                blended = cv2.addWeighted(roi, 0.5, sl_img, 0.5, 0)
                canvas[x:(x + h), y:(y + w)] = blended
            else:
                canvas[x:(x + h), y:(y + w)] = sl_img

            # draw bbox and label
            cv2.rectangle(canvas, (y, x), (y + w - 1, x + h - 1), (0, 255, 0), 2)
            cv2.putText(canvas, f"{idx}", (y + 6, x + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if out_path:
            cv2.imwrite(out_path, canvas)

        if wait:
            cv2.imshow("slices", canvas)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        return canvas

    def _visualize_detections(self, img: np.ndarray, detections: np.ndarray,
                              class_names: Dict[int, str] = None,
                              show_class_label: bool = False,
                              show_conf: bool = False,
                              conf_thresh: float = 0.05,
                              max_boxes: int = 300,
                              out_path: str = None,
                              title: str = "detections",
                              wait: bool = True,
                              fit_to_screen: bool = True) -> np.ndarray:
        """Draw detection boxes and optional labels onto a copy of ``img``.

        Parameters
        ----------
        img : np.ndarray
            HxWx3 uint8 BGR image to annotate. The original array is not
            modified; a copy is returned.
        detections : np.ndarray
            Nx6 array with rows ``(x1, y1, x2, y2, conf, cls)``. Coordinates
            may be absolute pixel values or normalized to [0,1].
        class_names : Dict[int,str], optional
            Mapping of class id to readable label used when ``show_class_label``
            is True.
        show_class_label : bool, optional
            If True draw the class label (or id) next to each box.
        show_conf : bool, optional
            If True append the confidence value to the label text.
        conf_thresh : float, optional
            Minimum confidence required to display a box.
        max_boxes : int, optional
            Maximum number of boxes to render.
        out_path : str, optional
            If provided the annotated image will be saved to this path.
        title : str, optional
            Window title used when displaying the image.
        wait : bool, optional
            If True, display a window and wait for a keypress before
            returning.
        fit_to_screen : bool, optional
            If True attempt to resize the display to fit the screen.

        Returns
        -------
        np.ndarray
            The annotated image (uint8 BGR).

        Raises
        ------
        ValueError
            If ``img`` is None or ``detections`` is not an (N,6) array.
        """

        if img is None:
            raise ValueError("img must be a valid image")

        annotated = img.copy()
        if detections is None or len(detections) == 0:
            if out_path:
                cv2.imwrite(out_path, annotated)
            return annotated

        dets = np.asarray(detections)
        if dets.ndim != 2 or dets.shape[1] < 6:
            raise ValueError("detections must be an array of shape (N,6)")

        # filter by confidence
        confs = dets[:, 4]
        keep = confs >= conf_thresh
        dets = dets[keep]
        if dets.shape[0] == 0:
            if out_path:
                cv2.imwrite(out_path, annotated)
            return annotated

        # cap boxes
        dets = dets[:max_boxes]

        h, w = annotated.shape[:2]

        coords = dets[:, :4].astype(float)
        # detect normalized coordinates (max <= 1.01)
        if coords.max() <= 1.01:
            coords[:, 0] *= w
            coords[:, 2] *= w
            coords[:, 1] *= h
            coords[:, 3] *= h

        # draw boxes
        for row in dets:
            x1, y1, x2, y2, conf, cls = row[:6]
            x1, y1, x2, y2 = int(round(x1)), int(round(y1)), int(round(x2)), int(round(y2))
            cls_id = int(round(cls))

            # color per class
            hue = (cls_id * 37) % 180
            color = tuple(int(c) for c in cv2.cvtColor(np.uint8([[[hue, 200, 200]]]), cv2.COLOR_HSV2BGR)[0, 0])

            # clamp coords
            x1 = max(0, min(w - 1, x1))
            x2 = max(0, min(w - 1, x2))
            y1 = max(0, min(h - 1, y1))
            y2 = max(0, min(h - 1, y2))

            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)

            label = class_names.get(cls_id, str(cls_id)) if class_names else str(cls_id)
            parts: List[str] = []
            if show_class_label:
                parts.append(label)
            if show_conf:
                parts.append(f"{conf:.2f}")

            if parts:
                label_text = " ".join(parts)
                (tw, th), _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                # place background box
                bx1, by1 = x1, max(0, y1 - th - 6)
                bx2, by2 = x1 + tw + 8, by1 + th + 6
                cv2.rectangle(annotated, (bx1, by1), (bx2, by2), color, -1)
                cv2.putText(annotated, label_text, (bx1 + 4, by2 - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # draw total count in top-right corner
        try:
            total_count = int(dets.shape[0])
        except Exception:
            total_count = 0

        count_text = f"Count: {total_count}"
        (ctw, cth), _ = cv2.getTextSize(count_text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
        pad_x = 8
        pad_y = 6
        bx2 = w - 6
        bx1 = max(0, bx2 - ctw - pad_x)
        by1 = 6
        by2 = by1 + cth + pad_y
        cv2.rectangle(annotated, (bx1, by1), (bx2, by2), (0, 0, 0), -1)
        cv2.putText(annotated, count_text, (bx1 + 4, by2 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        if out_path:
            cv2.imwrite(out_path, annotated)

        if wait:
            # Adapt display size to screen if requested
            display_img = annotated
            display_h, display_w = annotated.shape[:2]
            if fit_to_screen:
                try:
                    import tkinter as tk
                    root = tk.Tk()
                    root.withdraw()
                    screen_w = root.winfo_screenwidth()
                    screen_h = root.winfo_screenheight()
                    root.destroy()
                    max_w = int(screen_w * 0.92)
                    max_h = int(screen_h * 0.92)
                    scale = min(max_w / display_w, max_h / display_h, 1.0)
                    if scale < 1.0:
                        display_w = max(1, int(display_w * scale))
                        display_h = max(1, int(display_h * scale))
                        display_img = cv2.resize(annotated, (display_w, display_h), interpolation=cv2.INTER_AREA)
                except Exception:
                    pass

            cv2.namedWindow(title, cv2.WINDOW_NORMAL)
            try:
                cv2.resizeWindow(title, display_w, display_h)
            except Exception:
                pass
            cv2.imshow(title, display_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        return annotated
