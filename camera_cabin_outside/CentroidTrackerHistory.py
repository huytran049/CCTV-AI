import numpy as np
from scipy.spatial import distance as dist
from collections import OrderedDict
import logging
import os
import datetime as dt       #import datetime library

class CentroidTrackerHistory:
    def __init__(self, maxDisappeared=50):
        try:
            # Configure logging 
            self.logger = logging.getLogger(__name__)
            self.nextObjectID = 0
            self.objects = OrderedDict()
            self.disappeared = OrderedDict()
            self.maxDisappeared = maxDisappeared
            self.history = OrderedDict()
            self.boundingBoxes = OrderedDict()  # Store bounding boxes for each objectID
        except Exception as e:
            self.logger.exception("Exception in __init__")

    def register(self, centroid, bbox):
        try:
            self.objects[self.nextObjectID] = centroid
            self.disappeared[self.nextObjectID] = 0
            self.history[self.nextObjectID] = [centroid]
            self.boundingBoxes[self.nextObjectID] = bbox  # Initialize the bounding box
            self.nextObjectID += 1
        except Exception as e:
            self.logger.error('Error at %s', 'CentroidTrackerHistory', exc_info=e)     

    def deregister(self, objectID):
        try:
            del self.objects[objectID]
            del self.disappeared[objectID]
            del self.history[objectID]
            del self.boundingBoxes[objectID]  # Remove the bounding box
        except Exception as e:
            self.logger.error(f"Exception in deregister for objectID {objectID}", 'CentroidTrackerHistory', exc_info=e)    

    def update(self, rects):
        try:
            if len(rects) == 0:
                for objectID in list(self.disappeared.keys()):
                    self.disappeared[objectID] += 1
                    if self.disappeared[objectID] > self.maxDisappeared:
                        self.deregister(objectID)
                return self.objects

            inputCentroids = np.zeros((len(rects), 2), dtype="int")

            for (i, (startX, startY, endX, endY)) in enumerate(rects):
                cX = int((startX + endX) / 2.0)
                cY = int((startY + endY) / 2.0)
                inputCentroids[i] = (cX, cY)

            if len(self.objects) == 0:
                for i in range(0, len(inputCentroids)):
                    self.register(inputCentroids[i], rects[i])
            else:
                objectIDs = list(self.objects.keys())
                objectCentroids = list(self.objects.values())

                D = dist.cdist(np.array(objectCentroids), inputCentroids)

                rows = D.min(axis=1).argsort()
                cols = D.argmin(axis=1)[rows]

                usedRows = set()
                usedCols = set()

                for (row, col) in zip(rows, cols):
                    if row in usedRows or col in usedCols:
                        continue

                    objectID = objectIDs[row]
                    self.objects[objectID] = inputCentroids[col]
                    self.history[objectID].append(inputCentroids[col])
                    self.boundingBoxes[objectID] = rects[col]  # Update the bounding box
                    self.disappeared[objectID] = 0

                    usedRows.add(row)
                    usedCols.add(col)

                unusedRows = set(range(0, D.shape[0])).difference(usedRows)
                unusedCols = set(range(0, D.shape[1])).difference(usedCols)

                if D.shape[0] >= D.shape[1]:
                    for row in unusedRows:
                        objectID = objectIDs[row]
                        self.disappeared[objectID] += 1

                        if self.disappeared[objectID] > self.maxDisappeared:
                            self.deregister(objectID)
                else:
                    for col in unusedCols:
                        self.register(inputCentroids[col], rects[col])

            return self.objects
        except Exception as e:
            self.logger.error(f"Exception in update", 'CentroidTrackerHistory', exc_info=e)     

    def get_history(self, objectID):
        try:
            return self.history.get(objectID, [])
        except Exception as e:
            self.logger.error(f"Exception in get_history for objectID {objectID}", 'CentroidTrackerHistory', exc_info=e)     

    def get_current_bbox(self, objectID):
        try:
            return self.boundingBoxes.get(objectID, None)
        except Exception as e:
            self.logger.error(f"Exception in get_current_bbox for objectID {objectID}", 'CentroidTrackerHistory', exc_info=e)    